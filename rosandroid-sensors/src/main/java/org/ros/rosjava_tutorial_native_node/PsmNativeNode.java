package org.ros.rosjava_tutorial_native_node;

import org.ros.namespace.GraphName;
import org.ros.node.NativeNodeMain;

/**
 * Class to implement a polar scan matcher native node.
 **/
public class PsmNativeNode extends NativeNodeMain {
    private static final String libName = "movebase_jni";
    public static final String nodeName = "psm";

    public PsmNativeNode() {
        super(libName);
    }

    public PsmNativeNode(String[] remappingArguments) {
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
