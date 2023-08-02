package org.ollide.rosandroid;

import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.namespace.GraphName;
import org.ros.node.AbstractNodeMain;
import org.ros.node.ConnectedNode;
import org.ros.node.topic.Subscriber;

import nav_msgs.Path;

public class PathListenerNode extends AbstractNodeMain {

    public static String TAG = "path_listener";
    public static final String nodeName = "path_listener_node";
    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }

    @Override
    public void onStart(ConnectedNode node) {
        final Log log = node.getLog();
        Subscriber<Path> subscriber1 = node.newSubscriber("/move_base/DWAPlannerROS/local_plan", "nav_msgs/Path");
        Subscriber<Path> subscriber2 = node.newSubscriber("/move_base/DWAPlannerROS/global_plan", "nav_msgs/Path");
        Subscriber<Path> subscriber3 = node.newSubscriber("/move_base/NavfnROS/plan", "nav_msgs/Path");
        subscriber1.addMessageListener(new MessageListener<Path>() {
            @Override
            public void onNewMessage(Path message) {
                log.info("New local plan path is here!");
            }
        });
    }
}
