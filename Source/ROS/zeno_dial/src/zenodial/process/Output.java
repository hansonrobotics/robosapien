package zenodial.process;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.TreeMap;
import org.ros.node.topic.Publisher;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;
import zenodial.ros.ROSMaster;
import zenodial.rule.Then;

public class Output {
	private static Logger log = new Logger("Output", Level.NORMAL);
	
	public static Map.Entry<Double, Then> getOutput(HashMap<Double, Then> potentialOutputs) {
		TreeMap<Double, Then> sortedOutputs = new TreeMap<Double, Then>(Collections.reverseOrder());
		sortedOutputs.putAll(potentialOutputs);
		
		for (Map.Entry<Double, Then> entry : sortedOutputs.entrySet()) {
			if (Weighting.MUST != entry.getKey() && Weighting.PERVIOUS != entry.getKey()) { // Print only the utterances
				log.info("[" + new DecimalFormat("#.###").format(entry.getKey()) + "] " + entry.getValue().getThen());
			}
		}
		
		log.debug("Output = " + sortedOutputs.firstEntry().getValue());
		return sortedOutputs.firstEntry();
	}
	
	public static void publishOutput(String output) {
		int maxLength = 90; // Max no. of characters
		int timeToWait = 1500; // TODO: Should change it to a flag in practice
		
		try {
			// Split the output into shorter sentences if it's length > maxLength
			if (output.length() > maxLength) {
				ArrayList<String> sentences = new ArrayList<String>();
				sentences = splitUtterance(output, maxLength);
				
				for (String sentence : sentences) {
					publish(sentence);
					Thread.sleep(timeToWait);
				}
			}
			
			// Else if output.length < maxLength, publish it directly
			else {
				publish(output);
			}
		}
		
		catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
	
	private static void publish(String output) {
		// For command line
		System.out.println("[ZenoDial] " + output + "\n");
		
		// For ROS
		Publisher<std_msgs.String> publisher = ROSMaster.getPublisher(ROSMaster.rosPublisherTopic);
		std_msgs.String stringToPub = publisher.newMessage();
		stringToPub.setData(output);
		publisher.publish(stringToPub);
	}
	
	private static ArrayList<String> splitUtterance(String utterance, int maxLength) {
		ArrayList<String> sentences = new ArrayList<String>();

		while (utterance.length() > 0 && utterance.length() > maxLength) {
			int indexToSplit = checkAndSplit(utterance.replace('?', '.').replace('!', '.').replace(';', '.').replace("...", "."), ".", maxLength, true);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, ",", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " where ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " who ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " whom ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " and ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " but ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " or ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " if ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " then ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " so ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " also ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " moreover ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " by the way ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " because of ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " because ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " hence ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " therefore ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " however ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " in ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " on ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " at ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " when ", maxLength, false);
			if (indexToSplit == 0) indexToSplit = checkAndSplit(utterance, " ", maxLength, false);
			
			if (indexToSplit > 0 && indexToSplit < maxLength) {
				sentences.add(utterance.substring(0, indexToSplit + 1));
				utterance = utterance.substring(indexToSplit + 1).trim();
			}
			
			else {
				log.severe("Error!! Index = " + indexToSplit);
			}
		}
		
		sentences.add(utterance);
//		int count = 1;
//		for (String sentence : sentences) {
//			System.out.println("(" + count + ") " + sentence);
//			count++;
//		}
		
		return sentences;
	}
	
	private static int checkAndSplit(String strToSplit, String splitter, int maxLength, boolean getTheFirstOne) {
		int index = 0;
		
		if (getTheFirstOne) index = strToSplit.toLowerCase().indexOf(splitter);
		else index = strToSplit.toLowerCase().substring(0, maxLength).trim().lastIndexOf(splitter);
		
		if (index >= maxLength || index < 0) return 0;
		else return index;
	}
}
