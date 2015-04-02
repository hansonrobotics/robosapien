package zenodial.ros;

import org.ros.message.MessageListener;
import org.ros.node.topic.Subscriber;
import zenodial.log.Logger;
import zenodial.log.Logger.Level;
import zenodial.process.Input;
import zenodial.ros.ROSMaster;

public class ROSListener {
	private static Logger log = new Logger("ROSListener", Level.NORMAL);
	
	public static void createInputListener() {
		Subscriber<std_msgs.String> subscriber = ROSMaster.initializeSubscriber("ZenoDialListener", ROSMaster.rosSubscriberTopic);
		subscriber.addMessageListener(new MessageListener<std_msgs.String>() {
			@Override
			public void onNewMessage(std_msgs.String message) {
				String msgReceived = message.getData();
				log.info("Message reached: \"" + msgReceived + "\"");
				if (msgReceived.equals("BADINPUT")) return;
				Input.setInputUtterance(msgReceived);
			}
		});
	}
}
