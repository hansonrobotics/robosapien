package zenodial.rule;

public class Then {
	private String ruleSeqID;
	private String util;
	private double utilValue;
	private String topic;
	private String variable;
	private String then;
	
	public Then(String ruleSeqID, String util, String then) {
		this.ruleSeqID = ruleSeqID;
		this.util = util;
		this.topic = "general";
		this.variable = "";
		this.then = then;
	}
	
	public Then(String ruleSeqID, String util, String topic, String variable, String then) {
		this.ruleSeqID = ruleSeqID;
		this.util = util;
		this.topic = topic;
		this.variable = variable;
		this.then = then;
	}
	
	public String getRuleSeqID() {
		return ruleSeqID;
	}
	
	public String getUtil() {
		return util;
	}
	
	public double getUtilValue() {
		return utilValue;
	}
	
	public String getTopic() {
		return topic;
	}
	
	public String getVariable() {
		return variable;
	}
	
	public String getThen() {
		return then;
	}
}