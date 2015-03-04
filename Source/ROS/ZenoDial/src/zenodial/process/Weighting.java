package zenodial.process;

import java.util.Random;

public class Weighting {
	private static final double MIN = 0.01;
	private static final double NEXT = 10.0;
	public static final double PERVIOUS = -0.1;
	public static final double MUST = 1000.0;
	
	public static double get(String util) {
		double weighting = (util.matches("[+-]?\\d*(\\.\\d+)?"))? Double.parseDouble(util) : new Random().nextDouble();
		
		if (util.equals("random") || util.equals("next")) {
			weighting = new Random().nextDouble() + MIN;
		}
		
		else if (util.equals("previous")) {
			weighting = PERVIOUS;
		}
		
		else if (util.equals("nextone")) {
			weighting = NEXT;
		}
		
		else if (util.equals("must")) {
			weighting = MUST;
		}
		
		else if (util.equals("wolframAlpha")) {
			weighting = 100;
		}
		
		else if (util.equals("jmegahal")) {
			weighting = new Random().nextDouble() + MIN;
		}
		
		return weighting;
	}
}