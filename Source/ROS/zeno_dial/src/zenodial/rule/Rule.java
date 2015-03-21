package zenodial.rule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map.Entry;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;

public class Rule {
	private static Logger log = new Logger("Rule", Level.DEBUG);
	private String id;
	private boolean cache = true;
	private String condition;
	private ArrayList<String> rawConditions = new ArrayList<String>();
	private boolean hasVariableInCondition = false;
	private HashMap<Then, Double> effect = new HashMap<Then, Double>();
	
	public Rule(String id) {
		this.id = id;
	}
	
	public Rule(String id, boolean cache) {
		this.id = id;
		this.cache = cache;
	}
	
	public String getRuleID() {
		return id;
	}
	
	public boolean needToCache() {
		return cache;
	}
	
	public String getCondition() {
		return condition;
	}
	
	public void setCondition(String condition) {
		this.condition = condition;
	}
	
	public void addRawCondition(String condition) {
		rawConditions.add(condition);
	}
	
	public ArrayList<String> getRawCondition() {
		return rawConditions;
	}
	
	public boolean hasVariableInCondition() {
		return hasVariableInCondition;
	}
	
	public void hasVariableInCondition(boolean flag) {
		hasVariableInCondition = flag;
	}
	
	public HashMap<Then, Double> getEffect() {
		return effect;
	}
	
	public void addThen(Then then, double utilValue) {
		effect.put(then, utilValue);
	}
	
	public void printRule() {
		log.info("Rule ID: " + id);
		System.out.println("Condition:\n" + condition + "\nEffect:");
		
		Iterator<Entry<Then, Double>> iterator = effect.entrySet().iterator();
		while (iterator.hasNext()) {
			Entry<Then, Double> entry = iterator.next();
			System.out.print("[" + entry.getValue() + "] ");
			System.out.println(entry.getKey().getThen());
		}
	}
}