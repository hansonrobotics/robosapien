package zenodial.process;

import zenodial.ZenoDial;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;

public class Input {
	private static String inputUtterance = "";
	private static Logger log = new Logger("Output", Level.NORMAL);
	
	public static String getInputUtterance() {
		return inputUtterance;
	}
	
	public static void setInputUtterance(String input) {
		inputUtterance = Processor.removeUnwantedCharacters(input.trim(), false);
	}
	
	public static void resetInputUtterance() {
		inputUtterance = "";
	}
	
	public static boolean isUserUtterance(String input) {
		if ("#changeprofile".equals(input)) {
			log.debug("--> #changeprofile");
			ZenoDial.profile.clear();
		}
		
		else if (!"".equals(input)) {
			setInputUtterance(input);
			return true;
		}
		
		return false;
	}
}