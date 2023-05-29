## Rosjava Project Template for Android Studio ##

This project template makes it easy to get started with Android programming for
[ROS.org](http://www.ros.org/wiki/). Its structure complies with the new Gradle-based
build system and can be opened and assembled in Android Studio with no further changes.

### Project Structure ###

The example Android application can be found in [rosandroid-example-app](rosandroid-example-app).
It contains a simple Activity which extends RosActivity and starts a node publishing messages
on a ROS topic named ``time``. You can use all ROS Java components here because the rosandroid-core
is declared as a dependency ([libraries/rosandroid-core](libraries/rosandroid-core)). The
rosandroid-core consists of Android specific Java code only. All other dependencies (mainly rosjava
itself and common messages for ROS) are integrated through ROS's maven repository on GitHub. 

ROS automatically launches a MasterChooser activity to establish a connection to a running ROS
instance. This comes in handy for most developers but if you prefer a custom approach or design to
connect to your ROS master you need to change these classes, which is why I've included the ROS
Android source code.

### Requirements ###

This template has been tested with ROS Lunar, it may not work with older or newer versions of ROS.
Previously, it has successfully been used with ROS Hydro and Indigo.

It is optimized to work with the latest release of [Android Studio](https://developer.android.com/sdk/index.html) (currently 3.0.1) but can also be build from
the command-line. As this is plain Android/Java/Gradle no local installation of ROS is needed to
compile, therefore you can not only use Ubuntu but also Windows or Mac.

### Build
1. Build [ros_android](https://github.com/JzHuai0108/roscpp_android.git).
branch kinetic.

2. *This step can be skipped.* create the header chatter_jni.h for the ChatterNativeNode java class.
Following instructions [here](http://wiki.ros.org/android_ndk/Tutorials/WrappingNativeRosjavaNode).
```
javac -h . -cp .:/home/jhuai/.gradle/caches/modules-2/files-2.1/org.ros.rosjava_core/rosjava/0.3.5/7320e6ff76abf066da6850f44f3dcb05f890e043/rosjava-0.3.5.jar /media/jhuai/docker/roscpp_android_ndk/AndroidSensors/rosandroid-sensors/src/main/java/org/ros/rosjava_tutorial_native_node/ChatterNativeNode.java
```

3. Setup and build the roscpp_android_ndk static library
In a terminal
```
cd /media/jhuai/docker/roscpp_android_ndk/ros_android
./setup_ndk_project.sh /media/jhuai/docker/roscpp_android_ndk/ros_android/output 0
export PATH=$PATH:/home/jhuai/Android/android-ndk-r18b
cd /media/jhuai/docker/roscpp_android_ndk/ros_android/output/roscpp_android_ndk
ndk-build
```

4. Build the chatter_jni.so
```
# In a terminal
export PATH=$PATH:/home/jhuai/Android/android-ndk-r18b
cd /media/jhuai/docker/roscpp_android_ndk/AndroidSensors/rosandroid-sensors/src/main/jni
ndk-build
```
The jni folder's structure is
```
.
├── Android.mk
├── Application.mk
├── include
│   └── chatter_jni.h
└── src
    └── chatter_jni.cpp
```

5. Copy the dependency .so files from /roscpp_android_ndk/ros_android/output/target/lib/ to 
/roscpp_android_ndk/AndroidSensors/rosandroid-sensors/src/main/jniLibs/arm64-v8a/.

Then build the apk in Android studio. Run the app on an Android phone.
Check show advanced options. Click New public master.

6. Connect to android roscore
Android Public master URI can be found in logcat, e.g., http://192.168.2.134:11311/
Then, on ubuntu in a terminal
```
export ROS_MASTER_URI=http://192.168.2.134:11311
export ROS_IP=192.168.2.77  # This is the Ubuntu IP address in the local network which can be found by ifconfig
export ROS_HOSTNAME=192.168.2.77
```

7. Echo the messages on Ubuntu.
In the same terminal as above
```
rostopic list
rostopic echo /chatter
```

### Error messages
1. 2023-05-29 17:23:50.865 16880-16956/org.ollide.rosandroid E/RegistrationManagerImpl: Error during onNodeReplacement call
    java.lang.RuntimeException: java.net.ConnectException: failed to connect to /192.168.2.134 (port 39977) from /192.168.2.134 (port 41247) after 60000ms: isConnected failed: ECONNREFUSED (Connection refused)
        at org.ros.internal.node.xmlrpc.XmlRpcClientFactory$1.invoke(XmlRpcClientFactory.java:157)
        at java.lang.reflect.Proxy.invoke(Proxy.java:1006)
        at $Proxy4.shutdown(Unknown Source)
        at org.ros.internal.node.client.SlaveClient.shutdown(SlaveClient.java:62)
        at org.ros.internal.node.server.master.MasterServer.onNodeReplacement(MasterServer.java:485)
        at org.ros.internal.node.server.master.MasterRegistrationManagerImpl.obtainNodeRegistrationInfo(MasterRegistrationManagerImpl.java:431)
        at org.ros.internal.node.server.master.MasterRegistrationManagerImpl.registerPublisher(MasterRegistrationManagerImpl.java:97)
        at org.ros.internal.node.server.master.MasterServer.registerPublisher(MasterServer.java:217)
        at org.ros.internal.node.xmlrpc.MasterXmlRpcEndpointImpl.registerPublisher(MasterXmlRpcEndpointImpl.java:93)
        at java.lang.reflect.Method.invoke(Native Method)
        at org.apache.xmlrpc.server.ReflectiveXmlRpcHandler.invoke(ReflectiveXmlRpcHandler.java:115)
        at org.apache.xmlrpc.server.ReflectiveXmlRpcHandler.execute(ReflectiveXmlRpcHandler.java:106)
        at org.apache.xmlrpc.server.XmlRpcServerWorker.execute(XmlRpcServerWorker.java:46)
        at org.apache.xmlrpc.server.XmlRpcServer.execute(XmlRpcServer.java:86)
        at org.apache.xmlrpc.server.XmlRpcStreamServer.execute(XmlRpcStreamServer.java:200)
        at org.apache.xmlrpc.webserver.Connection.run(Connection.java:208)
        at org.apache.xmlrpc.util.ThreadPool$Poolable$1.run(ThreadPool.java:68)
     Caused by: java.net.ConnectException: failed to connect to /192.168.2.134 (port 39977) from /192.168.2.134 (port 41247) after 60000ms: isConnected failed: ECONNREFUSED (Connection refused)
        at libcore.io.IoBridge.isConnected(IoBridge.java:349)


2. 2023-05-29 17:43:43.617 18107-18247/org.ollide.rosandroid E/XmlRpcErrorLogger: No such handler: system.multicall
    org.apache.xmlrpc.server.XmlRpcNoSuchHandlerException: No such handler: system.multicall
        at org.apache.xmlrpc.server.AbstractReflectiveHandlerMapping.getHandler(AbstractReflectiveHandlerMapping.java:214)
        at org.apache.xmlrpc.server.XmlRpcServerWorker.execute(XmlRpcServerWorker.java:45)
        at org.apache.xmlrpc.server.XmlRpcServer.execute(XmlRpcServer.java:86)
        at org.apache.xmlrpc.server.XmlRpcStreamServer.execute(XmlRpcStreamServer.java:200)
        at org.apache.xmlrpc.webserver.Connection.run(Connection.java:208)
        at org.apache.xmlrpc.util.ThreadPool$Poolable$1.run(ThreadPool.java:68)

The above errors have been discussed [here](https://github.com/rosjava/rosjava_core/pull/273).

### Contribution ###

Feel free to contribute to this project by either raising issues or handing in pull requests.


### References
[1] [An detailed example on using jni libraries](https://medium.com/nerd-for-tech/guide-to-jni-java-native-interface-5b63fea01828)

[2] [Henry Henderson's answer for java.lang.UnsatisfiedLinkError: dlopen failed: library not found error](https://stackoverflow.com/questions/52076641/java-lang-unsatisfiedlinkerror-dlopen-failed-library-not-found)



