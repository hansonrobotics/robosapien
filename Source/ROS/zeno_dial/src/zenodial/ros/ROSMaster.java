package zenodial.ros;

import java.net.URI;
import java.util.ArrayList;
import java.util.HashMap;
import org.ros.concurrent.DefaultScheduledExecutorService;
import org.ros.node.ConnectedNode;
import org.ros.node.DefaultNodeFactory;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeListener;
import org.ros.node.topic.Publisher;
import org.ros.node.topic.Subscriber;

public class ROSMaster {
	// The ROS topic that publisher will be publishing on for the output
	public static String rosPublisherTopic = "/act/tts/set_text";

	// The ROS topic that the subscriber will be subscribing to for the text input
	public static String rosSubscriberTopic = "/ZenoDial/text_input";

	// The ROS_MASTER_URI
	private static final String ROSCORE_IP = "http://127.0.0.1:11311";

	private static HashMap<String, Subscriber<std_msgs.String>> subscribers = new HashMap<String, Subscriber<std_msgs.String>>();
	private static HashMap<String, Publisher<std_msgs.String>> publishers = new HashMap<String, Publisher<std_msgs.String>>();
	
	public static Subscriber<std_msgs.String> initializeSubscriber(String nodeName, String rosTopic) {
		NodeConfiguration config = NodeConfiguration.newPrivate();
		config.setMasterUri(URI.create(ROSCORE_IP));
		config.setDefaultNodeName(nodeName);
		
		ConnectedNode connectedNode = (ConnectedNode) new DefaultNodeFactory(new DefaultScheduledExecutorService()).newNode(config, new ArrayList<NodeListener>());
		Subscriber<std_msgs.String> subscriber = connectedNode.newSubscriber(rosTopic, std_msgs.String._TYPE);
		subscribers.put(rosTopic, subscriber);
		
		return subscriber;
	}
	
	public static Publisher<std_msgs.String> initializePublisher(String nodeName, String rosTopic) {
		NodeConfiguration config = NodeConfiguration.newPrivate();
		config.setMasterUri(URI.create(ROSCORE_IP));
		config.setDefaultNodeName(nodeName);
		
		ConnectedNode connectedNode = (ConnectedNode) new DefaultNodeFactory(new DefaultScheduledExecutorService()).newNode(config, new ArrayList<NodeListener>());
		Publisher<std_msgs.String> publisher = connectedNode.newPublisher(rosTopic, std_msgs.String._TYPE);
		publishers.put(rosTopic, publisher);
		
		return publisher;
	}
	
	public static Publisher<std_msgs.String> getPublisher(String rosTopic) {
		return publishers.get(rosTopic);
	}
}
