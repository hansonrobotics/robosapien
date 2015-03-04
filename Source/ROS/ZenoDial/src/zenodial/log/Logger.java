package zenodial.log;

import java.util.Calendar;

public class Logger {
	private String className;
	private Level level;
	public static enum Level {
		NONE,
		DEBUG,
		NORMAL
	}
	
	public Logger(String className, Level level) {
		this.className = className;
		this.level = level;
	}
	
	public void debug(String msg) {
		if (level == Level.DEBUG) System.out.println(getTime() + " [" + className + "] " + msg);
	}
	
	public void info(String msg) {
		if (level == Level.NORMAL || level == Level.DEBUG) System.out.println(getTime() + " [" + className + "] " + msg);
	}
	
	public void warning(String msg) {
		System.err.println(getTime() + " [" + className + "] " + msg);
	}
	
	public void severe(String msg) {
		System.err.println(getTime() + " [" + className + "] " + msg);
		System.exit(-1);
	}
	
	private String getTime() {
		return Calendar.getInstance().getTime().toString().substring(4, 19);
	}
}