package zenodial;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Properties;
import java.util.Scanner;
import zenodial.ros.ROSMaster;
import zenodial.ros.ROSListener;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;
import zenodial.plugin.JMegaHAL;
import zenodial.process.Input;
import zenodial.process.Processor;
import zenodial.rule.RuleInterpreter;

public class ZenoDial {
	private static Logger log = new Logger("ZenoDial", Level.NORMAL);
	public static Properties profile = new Properties();
	public static String topic = "general";
	public static boolean needsJMegaHAL = true;
	public static boolean needsWolframAlpha = false;
	
	@SuppressWarnings ("resource")
	public static void main(String[] args) {
		final Scanner cmdScanner = new Scanner(System.in);
		
		loadProfile("./config/Test.profile");
		
		// For ROS
		System.setProperty("org.apache.commons.logging.Log", "org.apache.commons.logging.impl.NoOpLog");
		ROSListener.createInputListener();
		ROSMaster.initializePublisher("ZenoDialPublisher", ROSMaster.rosPublisherTopic);
		while (true) {
			// Ask for the profile if there is none in the system
			while (profile.isEmpty()) {
				System.out.print("Profile: ");
				loadProfile("./config/" + cmdScanner.nextLine());
			}
			
			// Process the input whenever there is one
			// The input can be from command line or other speech-to-text modules
			if (!"".equals(Input.getInputUtterance())) {
			// || Input.isUserUtterance(cmdScanner.nextLine())
				Processor.processInput(Input.getInputUtterance());
				Input.resetInputUtterance();
			}
		}
	}
	
	private static void loadProfile(String filename) {
		String ruleFilesLocation = "./res/rules/";
		String jmegahalLocation = "./res/jmegahal/";
		String jmegahalBrainLocation = "./res/jmegahal/brain/";
		
		try {
			profile.load(new FileInputStream(filename));
			
			// Check if there are any core properties missing
			if (profile.getProperty("robotname") == null) throw new IllegalArgumentException("robotname");
			if (profile.getProperty("rule") == null) throw new IllegalArgumentException("rule");
			if (profile.getProperty("jmegahal") == null) throw new IllegalArgumentException("jmegahal");
			if (profile.getProperty("jmegahalBT") == null) throw new IllegalArgumentException("jmegahalBT");
			
			/************************* For rule files *************************/
			String[] ruleSets = profile.getProperty("rule").split(",");
			for (int i = 0; i < ruleSets.length; i++) {
				String path = ruleFilesLocation + ruleSets[i];
				if (new File(path).exists()) {
					log.info("Reading rules (" + path + ")...");
					RuleInterpreter.readRules(path);
				}
				
				else throw new IllegalArgumentException(path);
			}
			
			/************************* For JMegaHAL *************************/
			String[] jmegahalSets = profile.getProperty("jmegahal").split(",");
			// Load from the brain
			if ("b".equals(profile.getProperty("jmegahalBT").toLowerCase())) {
				String path = jmegahalBrainLocation + jmegahalSets[0];
				if (new File(path).exists()) {
					JMegaHAL.setBrainFile(path);
					JMegaHAL.initializeFromBrain(path);
				}
				
				else throw new IllegalArgumentException(path);
			}
			
			// Load from the training documents
			else if ("t".equals(profile.getProperty("jmegahalBT").toLowerCase())) {
				JMegaHAL.initialize();
				
				for (int i = 0; i < jmegahalSets.length; i++) {
					String path = jmegahalLocation + jmegahalSets[i];
					JMegaHAL.setBrainFile(jmegahalBrainLocation + profile.getProperty("robotname"));
					JMegaHAL.trainFromCorpus(path);
				}
			}
			
			else throw new IllegalArgumentException("jmegahalBT");
			
			log.info("Profile \"" + filename + "\" was loaded successfully.");
		}
		
		catch (IllegalArgumentException e) {
			log.warning(e.getMessage() + " is missing! Please edit the file and try again!");
			profile.clear();
		}
		
		catch (IOException e) {
			log.warning("File not found! Please check and try again!");
			profile.clear();
		}
	}
	
	public static void updateSystemVariable(String variable, String value) {
		log.debug("Updating variable: \"" + variable + "\" to \"" + value + "\"");
		profile.put(variable, value);
	}
}
