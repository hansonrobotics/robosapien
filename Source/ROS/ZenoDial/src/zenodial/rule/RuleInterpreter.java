package zenodial.rule;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Stack;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.w3c.dom.Document;
import org.w3c.dom.Node;
import org.w3c.dom.NodeList;
import org.xml.sax.SAXException;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;
import zenodial.process.Weighting;
import zenodial.rule.Rule;
import zenodial.rule.Then;

public class RuleInterpreter {
	private static Logger log = new Logger("RuleInterpreter", Level.NORMAL);
	private static final String INPUT_UTTERANCE = "IU";
	private static ArrayList<Rule> rules = new ArrayList<Rule>();
	private static HashMap<String, String> variables = new HashMap<String, String>();
	
	public static ArrayList<Rule> getAllTheRules() {
		return rules;
	}
	
	public static String getInputUtteranceLabel() {
		return INPUT_UTTERANCE;
	}
	
	public static void readRules(String filePath) {
		try {
			// Get all the rule XML files
			ArrayList<File> listOfFiles = new ArrayList<File>(Arrays.asList(new File(filePath).listFiles()));
			
			// Loop through each of the rule XML files
			for (File file : listOfFiles) {
				// Skip directory and any non-XML files
				if (file.isDirectory() || !file.getName().endsWith(".xml")) continue;
				
				log.info("Processing \"" + file.getName() + "\"...");
				Document xmlDocument = DocumentBuilderFactory.newInstance().newDocumentBuilder().parse(file.getPath());
				xmlDocument.getDocumentElement().normalize();
				NodeList ruleList = xmlDocument.getElementsByTagName("rule");
				
				// Loop through each of the <rule> in the rule XML file
				for (int i = 0; i < ruleList.getLength(); i++) {
					Node ruleNode = ruleList.item(i);
					log.debug("Currently reading rule: " + ruleNode.getAttributes().getNamedItem("id").getNodeValue());
					
					Rule rule;
					// If there is a cache attribute and the value is "no", create the rule accordingly
					if (ruleNode.getAttributes().getNamedItem("cache") != null && "no".equals(ruleNode.getAttributes().getNamedItem("cache").getNodeValue())) {
						rule = new Rule(ruleNode.getAttributes().getNamedItem("id").getNodeValue(), false);
					}
					// Else, default value for the cache attribute is true
					else {
						rule = new Rule(ruleNode.getAttributes().getNamedItem("id").getNodeValue());
					}
					rules.add(rule);
					
					// Get all the <case> in the rule
					NodeList caseList = ruleNode.getChildNodes();
					
					// Loop through each of the <case> in the rule
					for (int j = 0; j < caseList.getLength(); j++) {
						Node caseNode = caseList.item(j);
						
						// Get <case> and skip the unwanted nodes
						if (!caseNode.getNodeName().equals("#text") && !caseNode.getNodeName().equals("#comment")) {
							log.debug("Reading case no. " + (j + 1) / 2 + " in rule: " + ruleNode.getAttributes().getNamedItem("id").getNodeValue());
							
							// Get all the <condition> & <effect> under the <case>
							NodeList conditionEffectList = caseNode.getChildNodes();
							
							// Loop through each of the <condition> & <effect> under the <case>
							for (int k = 0; k < conditionEffectList.getLength(); k++) {
								
								// Get either <condition> or <effect> and skip the unwanted nodes
								if (!conditionEffectList.item(k).getNodeName().equals("#text") && !conditionEffectList.item(k).getNodeName().equals("#comment")) {
									// For <condition>
									if (conditionEffectList.item(k).getNodeName().toLowerCase().equals("condition")) {
										Node conditionNode = conditionEffectList.item(k);
										readCondition(conditionNode, rule);
									}
									
									// For <effect>
									else if (conditionEffectList.item(k).getNodeName().toLowerCase().equals("effect")) {
										Node effectNode = conditionEffectList.item(k);
										readEffect(effectNode, rule);
									}
									
									else {
										log.severe("Only <condition> or <effect> node is accepted. No <" + conditionEffectList.item(k).getNodeName() + ">");
									}
								}
							}
						}
					}
				}
			}
		}
		
		catch (ParserConfigurationException | SAXException | IOException e) {
			log.severe("Cannot parse the rule XML file.");
			e.printStackTrace();
		}
	}
	
	private static void readCondition(Node conditionNode, Rule rule) {
		String condition = "";
		Stack<String> conditionOperators = new Stack<String>();
		
		if (conditionNode.hasAttributes()) {
			// If there exist an operator attribute in <condition>, push it to the stack
			if (conditionNode.getAttributes().getNamedItem("operator") != null) {
				conditionOperators.push(getOperator(conditionNode.getAttributes().getNamedItem("operator").getNodeValue()));
			}
		}
		
		// Get all the nodes under <condition>, i.e. <if> || <and> || <or> || <not>
		NodeList list = conditionNode.getChildNodes();
		
		// Loop through each of the nodes under <condition>
		for (int i = 0; i < list.getLength(); i++) {
			if (!list.item(i).getNodeName().equals("#text") && !list.item(i).getNodeName().equals("#comment")) {
				// Extract <if>, with <and>, <or>, and <not> handling
				// For <if> that's not the last one in the list, +2 is to get rid of the # nodes
				if (i + 2 < list.getLength()) {
					// For the 1st <if> under <not>
					if (conditionOperators.peek().equals("not") && !condition.startsWith("!")) {
						condition = "!" + condition + readSubConditions(list.item(i), conditionOperators, rule) + " || ";
					}
					
					// For the rest of the <if> under <not>
					if (conditionOperators.peek().equals("not")) {
						condition = condition + readSubConditions(list.item(i), conditionOperators, rule) + " || ";
					}
					
					// For operators other than <not>
					else {
						condition += readSubConditions(list.item(i), conditionOperators, rule) + conditionOperators.peek();
					}
				}
				
				// For the last <if>
				else {
					condition += readSubConditions(list.item(i), conditionOperators, rule);
				}
			}
		}

		if (conditionOperators.size() > 0) conditionOperators.clear();
		log.debug("*** IF (" + condition + ")");
		
		// Add <condition> to the rule object
		rule.setCondition(condition);
	}
	
	private static String readSubConditions(Node node, Stack<String> conditionOperators, Rule rule) {
		String subCondition = "";
		
		// If it's a <if>
		if (node.getNodeName().toLowerCase().equals("if")) {
			// Store the raw conditions for "variable extraction" later
			rule.addRawCondition(node.getTextContent());
			
			// For <if> containing a variable attribute
			if (node.getAttributes().getNamedItem("variable") != null && node.getAttributes().getNamedItem("relation") != null) {
				subCondition = normalizeCondition("{" + node.getAttributes().getNamedItem("variable").getNodeValue() + "}", node.getAttributes().getNamedItem("relation").getNodeValue(), node.getTextContent());
				
				// Add the variable and its value into the variables hash map
				variables.put(node.getAttributes().getNamedItem("variable").getNodeValue(), node.getTextContent());
			}
			
			// For <if> containing only the relation attribute
			else if (node.getAttributes().getNamedItem("relation") != null) {
				subCondition = normalizeCondition("{" + INPUT_UTTERANCE + "}", node.getAttributes().getNamedItem("relation").getNodeValue(), node.getTextContent());
			}
		}
		
		// If it's not a <if>, push the operator to the operator stack and then repeat the process recursively until there is an <if>
		else {
			conditionOperators.push(getOperator(node.getNodeName().toLowerCase()));
			subCondition += "(";
			
			for (int i = 0; i < node.getChildNodes().getLength(); i++) {
				if (!node.getChildNodes().item(i).getNodeName().equals("#text") && !node.getChildNodes().item(i).getNodeName().equals("#comment")) {
					// For <if> that's not the last one in the list, +2 is to get rid of the # nodes
					if (i + 2 < node.getChildNodes().getLength()) {
						// For the 1st <if> under <not>
						if (conditionOperators.peek().equals("not") && !subCondition.startsWith("!")) {
							subCondition = "!" + subCondition + readSubConditions(node.getChildNodes().item(i), conditionOperators, rule) + " || ";
						}
						
						// For the rest of the <if> under <not>
						else if (conditionOperators.peek().equals("not")) {
							subCondition = subCondition + readSubConditions(node.getChildNodes().item(i), conditionOperators, rule) + " || ";
						}
						
						// For operators other than <not>
						else {
							subCondition += readSubConditions(node.getChildNodes().item(i), conditionOperators, rule) + conditionOperators.peek();
						}
					}
					
					// For the last <if>
					else {
						subCondition += readSubConditions(node.getChildNodes().item(i), conditionOperators, rule) + ")";
						conditionOperators.pop();
					}
				}
			}
		}
		
		return subCondition;
	}
	
	private static void readEffect(Node effectNode, Rule rule) {
		String effect = "";
		NodeList thenList = effectNode.getChildNodes();
		
		// Loop through each of the <then>
		for (int i = 0; i < thenList.getLength(); i++) {
			Node thenNode = thenList.item(i);
			String util = "";
			String variableName = "";
			String topic = "general";
			
			if (!thenNode.getNodeName().equals("#text") && !thenNode.getNodeName().equals("#comment")) {
				if (thenNode.hasAttributes()) {
					// Get the util attribute
					if (thenNode.getAttributes().getNamedItem("util") != null) {
						util = thenNode.getAttributes().getNamedItem("util").getNodeValue();
						effect += "[" + util + "] ";
					}
					
					// For <then> that is used to assign value to a variable
					else if (thenNode.getAttributes().getNamedItem("variable") != null) {
						variableName = thenNode.getAttributes().getNamedItem("variable").getNodeValue();
						util = "must";
						effect += "{" + variableName + "} = ";
					}
					
					// For <then> that refers to a topic
					else if (thenNode.getAttributes().getNamedItem("topic") != null) {
						util = "must";
						effect += "[" + util + "] ";
					}
					
					else log.severe("\"util\" not found!");
					
					// Get the topic attribute
					if (thenNode.getAttributes().getNamedItem("topic") != null) {
						topic = thenNode.getAttributes().getNamedItem("topic").getNodeValue();
						effect += "<" + topic + "> ";
					}
				}
				
				// For printing out the effect only
				if (i + 2 < thenList.getLength()) effect += thenNode.getTextContent() + " ";
				else effect += thenNode.getTextContent();
				
				// Create an object for <then>
				Then then = new Then(rule.getRuleID() + "-" + (i + 1) / 2, util, topic, variableName, thenNode.getTextContent());
				
				// Check if there is a <then> following a "next"
				if ("next".equals(util) && (i + 2 == thenList.getLength())) log.severe("There has to be another <then> following a \"next\" <then>");
				
				// Add <then> to the rule object
				else rule.addThen(then, Weighting.get(then.getUtil()));
			}
		}
		
		log.debug("*** THEN (" + effect + ")");
	}
	
	private static String getOperator(String inputOpt) {
		String operator = "";
		
		if (inputOpt.trim().equals("and")) {
			operator = " && ";
		}
		
		else if (inputOpt.trim().equals("or")) {
			operator = " || ";
		}
		
		else if (inputOpt.trim().equals("not")) {
			operator = "not";
		}
		
		else if ("".equals(inputOpt.trim())) log.severe("relation attribute cannot be empty.");
		else log.severe("No such relation: " + inputOpt);
		
		return operator;
	}
	
	private static String normalizeCondition(String variable, String operator, String content) {
		String normalCondition = "";
		content = content.toLowerCase();
		
		if (content.contains(" * ")) {
			normalCondition = "\"" + variable + "\".match(/\\b" + content.replace(" * ", "\\b.*\\b") + "\\b/) !== null";
		}
		
		else if ("*".equals(content)) {
			normalCondition = "\"" + variable + "\" !== null";
		}
		
		else if ("=".equals(operator)) {
			normalCondition = "\"\\b" + variable + "\\b\" === \"\\b" + content + "\\b\"";
		}
		
		else if ("!=".equals(operator)) {
			normalCondition = "\"\\b" + variable + "\\b\" !== \"\\b" + content + "\\b\"";
		}
		
		else if ("in".equals(operator)) {
			normalCondition = "\"" + variable + "\".match(/\\b" + content + "\\b/) !== null";
		}
		
		else if ("!in".equals(operator)) {
			normalCondition = "\"" + variable + "\".match(/\\b" + content + "\\b/) === null";
		}
		
		else if ("sw".equals(operator)) {
			normalCondition = "\"" + variable + "\".match(/^" + content + "\\b/) !== null";
		}
		
		else if ("ew".equals(operator)) {
			// Assume every utterances end with a punctuation
			normalCondition = "(\"" + variable + "\".lastIndexOf(\"" + content + "\") >= 0 && \"" + variable + "\".lastIndexOf(\"" + content + "\") == \"" + variable + "\".length - \"" + content + "\".length)";
		}
		
		else {
			normalCondition = "\"" + variable + "\" " + operator + " \"" + content + "\"";
		}
		
		return normalCondition;
	}
	
	public static Rule getRule(String ruleID) {
		for (Rule rule : rules) {
			if (ruleID.equals(rule.getRuleID())) {
				return rule;
			}
		}
		
		log.warning("No such rule: " + ruleID);
		return null;
	}
}