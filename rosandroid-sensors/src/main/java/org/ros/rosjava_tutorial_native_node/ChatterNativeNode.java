package org.ros.rosjava_tutorial_native_node;

import org.ros.node.NativeNodeMain;
import org.ros.namespace.GraphName;

/**
 * Class to implement a chatter native node.
 **/
public class ChatterNativeNode extends NativeNodeMain {
    private static final String libName = "chatter_jni";
    public static final String nodeName = "chatter";

    public ChatterNativeNode() {
        super(libName);
    }

    public ChatterNativeNode(String[] remappingArguments) {
        super(libName, remappingArguments);
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of(nodeName);
    }

    @Override
    protected native int execute(String rosMasterUri, String rosHostname, String rosNodeName, String[] remappingArguments);

    @Override
    protected native int shutdown();
}
