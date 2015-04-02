package zenodial.process;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Random;
import java.util.Map.Entry;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.apache.commons.lang.StringUtils;
import zenodial.rule.Rule;
import zenodial.rule.RuleInterpreter;
import zenodial.rule.Then;
import zenodial.ZenoDial;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;
import zenodial.plugin.JMegaHAL;
import zenodial.plugin.WolframAlpha;
import zenodial.rule.RuleMatcher;

public class Processor {
	private static Logger log = new Logger("Processor", Level.NORMAL);
	private static HashMap<Double, Then> potentialOutputs = new HashMap<Double, Then>();
	private static ArrayList<String> somethingToSayWhenWaiting = new ArrayList<String>(Arrays.asList("Well", "Interesting", "That's a good question", "Let me think"));
	private static ArrayList<String> prefixes = new ArrayList<String>(Arrays.asList("I think it's ", "I heard that's ", "I believe it's ", "It should be "));
	private static ArrayList<Rule> matchingRules = new ArrayList<Rule>();
	private static Map.Entry<Double, Then> output;
	private static String lastOutputUtterance = "";
	public static boolean nothingSpokenYet = false;
	
	public static void processInput(String inputUtterance) {
		nothingSpokenYet = true;
		
		saySomethingWhenWaiting();
		
		// Get all the rules with conditions that satisfy the input utterance
		matchingRules = RuleMatcher.findMatchingRules(inputUtterance);
		putToPotentialOutputs(matchingRules);
		
		// Get the reply from JMegaHAL
		if (ZenoDial.needsJMegaHAL) {
			potentialOutputs.put(Weighting.get("jmegahal"), new Then("JMegaHAL", "jmegahal", JMegaHAL.getReply(inputUtterance)));
		}

		// Get the reply from Wolfram|Alpha
		// TODO: Add timeout
		// TODO: Better to check if it's a question
		if (ZenoDial.needsWolframAlpha) {
			String waReply = WolframAlpha.getReply(inputUtterance);
			if (!"".equals(waReply)) potentialOutputs.put(Weighting.get("wolframAlpha"), new Then("Wolfram|Alpha", "wolframAlpha", prefixes.get(new Random().nextInt(prefixes.size())) + waReply));
		}
		
		// Get the output
		output = Output.getOutput(potentialOutputs);
		
		// Replace the variables in the output with their actual values, if any
		String outputUtterance = processOutput(output.getValue());
		lastOutputUtterance = outputUtterance;
		
		// Publish the output
		nothingSpokenYet = false;
		Output.publishOutput(outputUtterance);
		
		// Update the topic
		ZenoDial.topic = output.getValue().getTopic();
		log.debug("Topic changes to \"" + ZenoDial.topic + "\"");

		contructCache(matchingRules, output);
		potentialOutputs.clear();
	}
	
	private static void saySomethingWhenWaiting() {
		Thread saySomethingThread = new Thread() {
			public void run() {
				try {
					Thread.sleep(5000);
					
					while (Processor.nothingSpokenYet) {
						String utterance = somethingToSayWhenWaiting.get(new Random().nextInt(somethingToSayWhenWaiting.size()));
						Output.publishOutput(utterance);
						Thread.sleep(5000);
					}

					return;
				}
				
				catch (InterruptedException e) {
					// e.printStackTrace();
					return;
				}
			}
		}; saySomethingThread.start();
	}
	
	private static void putToPotentialOutputs(ArrayList<Rule> matchingRules) {
		for (Rule rule : matchingRules) {
			Iterator<Entry<Then, Double>> iterator = rule.getEffect().entrySet().iterator();
			while (iterator.hasNext()) {
				Entry<Then, Double> entry = iterator.next();
				potentialOutputs.put(entry.getValue(), entry.getKey());
				log.debug("***************** [" + entry.getValue() + "] " + entry.getKey().getThen());
			}
		}
	}
	
	private static String processOutput(Then outputThenObj) {
		String finalOutput = removeUnwantedCharacters(outputThenObj.getThen(), false);
		
		if ("".equals(finalOutput)) {
			// For "@repeat" tag
			if ("@repeat".equals(outputThenObj.getTopic())) {
				finalOutput = lastOutputUtterance;
			}
			
			// If it's a topic redirection
			else if (!"".equals(outputThenObj.getTopic())) {
				potentialOutputs.clear();
				matchingRules.clear();
				Rule goToRule = RuleInterpreter.getRule(outputThenObj.getTopic());
				matchingRules.add(goToRule);
				putToPotentialOutputs(matchingRules);
				output = Output.getOutput(potentialOutputs);
				finalOutput = removeUnwantedCharacters(output.getValue().getThen(), false);
			}
		}
		
		if (finalOutput.contains("{") && finalOutput.contains("}")) {
			int numOfVariables = StringUtils.countMatches(finalOutput, "{");
			
			// Loop through the variables one by one and replace them with the appropriate values
			for (int i = 0; i < numOfVariables; i++) {
				Matcher matcher = Pattern.compile("\\{(.*?)\\}").matcher(finalOutput);
				if (matcher.find()) finalOutput = finalOutput.replace("{" + matcher.group(1) + "}", ZenoDial.profile.getProperty(matcher.group(1)));
				else log.warning("Unknown error! No variable found?");
			}
		}
		
		return finalOutput;
	}
	
	private static void contructCache(ArrayList<Rule> matchingRules, Map.Entry<Double, Then> output) {
		// Update the util value for each of the satisfied rules
		for (Rule rule : matchingRules) {
			// Skip those no-need-to-cache rules
			if (!rule.needToCache()) continue;
			
			log.debug("Constructing cache for rule: " + rule.getRuleID());
			Iterator<Entry<Then, Double>> iterator = rule.getEffect().entrySet().iterator();
			int nextSeqID = 0; // To store the sequence ID of that "next" utterance
			while (iterator.hasNext()) {
				Entry<Then, Double> entry = iterator.next();
				
				// Find the output utterance
				String outputUtterance = output.getValue().getThen();
				String ruleUtterance = entry.getKey().getThen();
				double outputUtil = output.getKey();
				double ruleUtil = entry.getValue();
				if (outputUtterance.equals(ruleUtterance) && ruleUtil == outputUtil) {
					// Reset the util value for the output utterance
					rule.getEffect().put(entry.getKey(), Weighting.get(entry.getKey().getUtil()));
					
					// Update the topic if needed
//					if (needToUpdateTopic) ZenoDial.topic = entry.getKey().getTopic();
					
					// If the output utterance has a "next" util, adjust the util value of its following one with "previous" util
					if ("next".equals(entry.getKey().getUtil())) {
						nextSeqID = Integer.parseInt(entry.getKey().getRuleSeqID().split("-")[1]);
					}
				}
				
				// Increase the util value by 1 for the unspoken utterances
				else if (entry.getValue() > 0) {
					rule.getEffect().put(entry.getKey(), entry.getValue() + 1);
				}
			}
			
			// If the util of output utterance = "next", update the util value of its "previous" utterance accordingly
			if (nextSeqID > 0) {
				iterator = rule.getEffect().entrySet().iterator();
				
				while (iterator.hasNext()) {
					Entry<Then, Double> entry = iterator.next();
					
					if (Integer.parseInt(entry.getKey().getRuleSeqID().split("-")[1]) == nextSeqID + 1) {
						rule.getEffect().put(entry.getKey(), Weighting.get("nextone"));
						log.debug("\"Next\" utterance of the same topic will be: \"" + entry.getKey().getThen() + "\"");
					}
				}
			}
		}
	}
	
	public static String removeUnwantedCharacters(String inputString, boolean needToRemovePunctuation) {
		String rtnString = inputString.replace("’", "'").replace("”", "\"").replace("‘", "'").replace("“", "\"").replace("~~", " around ").replaceAll("^~", "").replaceAll("  ", " ").trim();
		
		if (needToRemovePunctuation) return removePunctuation(rtnString);
		else return rtnString;
	}
	
	public static String removePunctuation(String inputString) {
		// If the inputString contains a punctuation at the end of the sentence, remove it
		if (",.?!~".contains(inputString.substring(inputString.length() - 1))) {
			return inputString.substring(0, inputString.length() - 1);
		}
		
		return inputString;
	}
}
