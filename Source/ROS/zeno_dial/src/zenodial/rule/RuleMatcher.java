package zenodial.rule;

import java.util.ArrayList;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.script.ScriptEngine;
import javax.script.ScriptEngineManager;
import javax.script.ScriptException;
import org.apache.commons.lang.StringUtils;
import zenodial.ZenoDial;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;
import zenodial.rule.Rule;

public class RuleMatcher {
	private static Logger log = new Logger("RuleMatcher", Level.NORMAL);
	
	public static ArrayList<Rule> findMatchingRules(String inputUtterance) {
		ScriptEngine scriptEngine = new ScriptEngineManager().getEngineByName("JavaScript");
		ArrayList<Rule> matchedRules = new ArrayList<Rule>();
		ArrayList<Rule> rulesToRemove = new ArrayList<Rule>();
		String matchedRuleIDs = "";
		
		try {
			// Get the rules with conditions that satisfy the user utterance
			for (Rule rule : RuleInterpreter.getAllTheRules()) {
				if ((boolean) scriptEngine.eval(processConditionVariable(rule, inputUtterance))) {
					matchedRules.add(rule);
					
					// Store the variables in the condition, if any
					if (rule.hasVariableInCondition()) {
						// Replace with the input utterance
						String condition = rule.getCondition().replace("{" + RuleInterpreter.getInputUtteranceLabel() + "}", "\"" + inputUtterance + "\"");
						
						// Check if there are any other variables that need to be stored in the system
						if (StringUtils.countMatches(condition, "{") > 0) {
							// Loop through each of the raw conditions to find the actual "matching" condition, and then extract the variable values from the input utterances
							for (String rawCondition : rule.getRawCondition()) {
								int rawConditionCount = 1;
								Matcher conditionMatcher = Pattern.compile(rawCondition.replaceAll("\\{.+?\\}", "(.+)").replaceAll("\\s\\*\\s", ".*")).matcher(inputUtterance);
								// Extract the variable values here if it's a match
								if (conditionMatcher.find()) {
									Matcher variableNameMatcher = Pattern.compile("\\{(.*?)\\}").matcher(rawCondition);
									while (variableNameMatcher.find()) {
										ZenoDial.updateSystemVariable(variableNameMatcher.group(1), conditionMatcher.group(rawConditionCount));
										rawConditionCount++;
									}
								}
							}
						}
					}
					
					if ("".equals(matchedRuleIDs)) matchedRuleIDs += rule.getRuleID();
					else matchedRuleIDs += "-" + rule.getRuleID();
					
					log.debug("YESSSSSS! Rule (" + rule.getRuleID() + ")");
				}
				
				else {
					log.debug("NOOOOOOO! Rule (" + rule.getRuleID() + ")");
				}
			}
		}
		
		catch (ScriptException e) {
			e.printStackTrace();
			log.severe("Error occurred when selecting the satisfied rules.");
		}
		
		// ==================================== Filter the output utterances base on the criteria: #On-topic > On-topic > Off-topic > General ==================================== //
		log.debug("Current topic = " + ZenoDial.topic);
		if ("".equals(matchedRuleIDs)) log.warning("No rules matched!!");
		else log.debug("Matched Rule IDs = " + matchedRuleIDs);
		
		// If there are on-topic-# utterances, remove all of the off-topic-# utterances, if any
		if (matchedRuleIDs.contains("#")) {
			for (Rule rule : matchedRules) {
				if (rule.getRuleID().contains("#") && !rule.getRuleID().contains(ZenoDial.topic)) {
					log.debug("Have off-topic-# utterances, removing: " + rule.getRuleID());
					rulesToRemove.add(rule);
					matchedRuleIDs = removeIDFromString(matchedRuleIDs, rule.getRuleID());
					log.debug("New Matched Rule IDs = " + matchedRuleIDs);
				}
			}
			
			matchedRules.removeAll(rulesToRemove);
			rulesToRemove.clear();
		}
		
		// If topic = "general"
		if ("general".equals(ZenoDial.topic)) { 
			// If there are any non-general utterances, remove all the general utterances
			if (StringUtils.countMatches(matchedRuleIDs, "general") <= StringUtils.countMatches(matchedRuleIDs, "-")) {
				for (Rule rule : matchedRules) {
					if (rule.getRuleID().contains("general")) {
						log.debug("Have non-general utterances, removing: " + rule.getRuleID());
						rulesToRemove.add(rule);
						matchedRuleIDs = removeIDFromString(matchedRuleIDs, rule.getRuleID());
						log.debug("New Matched Rule IDs = " + matchedRuleIDs);
					}
				}
			}
			
			matchedRules.removeAll(rulesToRemove);
			rulesToRemove.clear();
		}
		
		// Else if topic != "general"
		else {
			// If there are on-topic utterances, remove all off-topic and general utterances
			if (matchedRuleIDs.contains(ZenoDial.topic)) {
				for (Rule rule : matchedRules) {
					if (!rule.getRuleID().contains(ZenoDial.topic)) {
						log.debug("Have on-topic utterances, removing: " + rule.getRuleID());
						rulesToRemove.add(rule);
						matchedRuleIDs = removeIDFromString(matchedRuleIDs, rule.getRuleID());
						log.debug("New Matched Rule IDs = " + matchedRuleIDs);
					}
				}
			}
			
			// If there are no on-topic utterances, keep all off-topic utterances and remove all general utterances
			else if (!matchedRuleIDs.contains(ZenoDial.topic) && StringUtils.countMatches(matchedRuleIDs, "general") <= StringUtils.countMatches(matchedRuleIDs, "-")) {
				for (Rule rule : matchedRules) {
					if (rule.getRuleID().contains("general")) {
						log.debug("Have off-topic utterances, removing: " + rule.getRuleID());
						rulesToRemove.add(rule);
						matchedRuleIDs = removeIDFromString(matchedRuleIDs, rule.getRuleID());
						log.debug("New Matched Rule IDs = " + matchedRuleIDs);
					}
				}
			}
			
			// If there are no on-topic and off-topic utterances, keep only general utterances
			else {
				for (Rule rule : matchedRules) {
					if (!rule.getRuleID().contains("general")) {
						log.debug("Have only general utterances, removing: " + rule.getRuleID());
						rulesToRemove.add(rule);
						matchedRuleIDs = removeIDFromString(matchedRuleIDs, rule.getRuleID());
						log.debug("New Matched Rule IDs = " + matchedRuleIDs);
					}
				}
			}
			
			matchedRules.removeAll(rulesToRemove);
			rulesToRemove.clear();
		}
		
		// If there are general utterances other than "generalnothingtosay" and "generalquestions", select them
//		if (!"".equals(matchedRuleIDs.replace("generalquestions", "").replace("generalnothingtosay", "").replaceAll("-", ""))) {
//			for (Rule rule : matchedRules) {
//				if ("generalnothingtosay".equals(rule.getRuleID()) || "generalquestions".equals(rule.getRuleID())) {
//					log.debug("Have other general utterances, removing: " + rule.getRuleID());
//					rulesToRemove.add(rule);
//					matchedRuleIDs = matchedRuleIDs.replace("-" + rule.getRuleID(), "").replace(rule.getRuleID() + "-", "");
//					log.debug("New Matched Rule IDs = " + matchedRuleIDs);
//				}
//			}
//			
//			matchedRules.removeAll(rulesToRemove);
//			rulesToRemove.clear();
//		}
		
		// If there are only "generalnothingtosay" and "generalquestions", remove "generalnothingtosay"
//		else if (matchedRuleIDs.contains("generalquestions")) {
//			for (Rule rule : matchedRules) {
//				if ("generalnothingtosay".equals(rule.getRuleID())) {
//					log.debug("Have generalquestions utterances, removing: " + rule.getRuleID());
//					rulesToRemove.add(rule);
//					matchedRuleIDs = matchedRuleIDs.replace("-" + rule.getRuleID(), "").replace(rule.getRuleID() + "-", "");
//					log.debug("New Matched Rule IDs = " + matchedRuleIDs);
//				}
//			}
//			
//			matchedRules.removeAll(rulesToRemove);
//			rulesToRemove.clear();
//		}
		
		// If there are no answers from the rules for that question, consult the QA Engine(s)
//		if (matchedRuleIDs.contains("generalquestions")) {
//			ZenoDial.needWolframAlpha = true;
//		}
//		if (matchedRuleIDs.contains("generalnothingtosay")) {
//			ZenoDial.needJMegaHAL = true;
//		}
		
		if (matchedRuleIDs.equals("")) matchedRuleIDs = "<None>";
		log.info("Matched rules: " + matchedRuleIDs);
		log.debug("No. of matched rules: " + matchedRules.size());
		return matchedRules;
	}
	
	private static String processConditionVariable(Rule rule, String userUtterance) {
		String condition = rule.getCondition();
		
		// Replace input utterance variable to the actual value of input utterance before going to the script engine
		if (condition.contains("{" + RuleInterpreter.getInputUtteranceLabel() + "}")) {
			condition = condition.replace("{" + RuleInterpreter.getInputUtteranceLabel() + "}", userUtterance.toLowerCase());
		}
		
		// Replace other variables in the condition to wildcard for later processing
		if (condition.contains("{") && condition.contains("}")) {
			rule.hasVariableInCondition(true);
			condition = condition.replaceAll("\\s*\\{.+?\\}\\s*", ".*");
		}
		
		log.debug("Evaluating (" + rule.getRuleID() + "): " + condition);
		return condition;
	}
	
	private static String removeIDFromString(String strToProcess, String strToRemove) {
		String rtnStr = strToProcess.replace("-" + strToRemove, "").replace(strToRemove + "-", "");
		if (rtnStr.length() == strToProcess.length()) rtnStr = "";
		
		return rtnStr;
	}
}