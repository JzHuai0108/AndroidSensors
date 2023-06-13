package org.ros.rosjava_tutorial_native_node;

import org.ros.node.NativeNodeMain;
import org.ros.namespace.GraphName;

/**
 * Class to implement a movebase native node.
 **/
public class MoveBaseNativeNode extends NativeNodeMain {
    private static final String libName = "movebase_jni";
    public static final String nodeName = "movebase";

    public MoveBaseNativeNode() {
        super(libName);
    }

    public MoveBaseNativeNode(String[] remappingArguments) {
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
