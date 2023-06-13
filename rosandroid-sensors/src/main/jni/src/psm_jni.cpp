#include <android/log.h>
#include <ros/ros.h>

#include "psm_jni.h"
#include "psm_node.h"

#include "sensor_msgs/Imu.h"
#include "std_msgs/String.h"
#include <sstream>

using namespace std;

extern void log(const char *msg, ...);

extern inline string stdStringFromjString(JNIEnv *env, jstring java_string);


int loop_count_ = 0;

extern JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved);

void poseCallback(const geometry_msgs::Pose2DConstPtr& msg) {
    // ROS_INFO("%s", msg->data.c_str());
    loop_count_++;
    std_msgs::String msgo;
    std::stringstream ss;
    ss << "Pose2D data " << loop_count_ << " from ndk: x: "
        << msg->x << ", y: "
        << msg->y << ", theta: "
        << msg->theta << "." ;
    msgo.data = ss.str();
    log(msgo.data.c_str());
}

JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_PsmNativeNode_execute(
        JNIEnv *env, jobject obj, jstring rosMasterUri, jstring rosHostname, jstring rosNodeName,
        jobjectArray remappingArguments) {
    log("Native psm node started.");

    string master("__master:=" + stdStringFromjString(env, rosMasterUri));
    string hostname("__ip:=" + stdStringFromjString(env, rosHostname));
    string node_name(stdStringFromjString(env, rosNodeName));

    log(master.c_str());
    log(hostname.c_str());

    // Parse remapping arguments
    log("Before getting size");
    jsize len = env->GetArrayLength(remappingArguments);
    log("After reading size");

    std::string ni = "psm_jni";

    int argc = 0;
    const int static_params = 4;
    char **argv = new char *[static_params + len];
    argv[argc++] = const_cast<char *>(ni.c_str());
    argv[argc++] = const_cast<char *>(master.c_str());
    argv[argc++] = const_cast<char *>(hostname.c_str());

    //Lookout: ros::init modifies argv, so the references to JVM allocated strings must be kept in some other place to avoid "signal 11 (SIGSEGV), code 1 (SEGV_MAPERR), fault addr deadbaad"
    // when trying to free the wrong reference ( see https://github.com/ros/ros_comm/blob/indigo-devel/clients/roscpp/src/libros/init.cpp#L483 )
    char **refs = new char *[len];
    for (int i = 0; i < len; i++) {
        refs[i] = (char *) env->GetStringUTFChars(
                (jstring) env->GetObjectArrayElement(remappingArguments, i), NULL);
        argv[argc] = refs[i];
        argc++;
    }

    log("Initiating ROS...");
    ros::init(argc, &argv[0], node_name.c_str());
    log("ROS intiated.");

    // Release JNI UTF characters
    for (int i = 0; i < len; i++) {
        env->ReleaseStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, i),
                                   refs[i]);
    }
    delete refs;
    delete argv;

    ros::NodeHandle n("psm_chat");
    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
    ros::Subscriber sub = n.subscribe("pose2D", 100, poseCallback);
    ros::Rate loop_rate(100);

    PSMNode psmNode;

    int count = 0;
    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;
        std::stringstream ss;
        ss << "psm hello world " << count;
        msg.data = ss.str();
        ROS_INFO("%s", msg.data.c_str());

        /**
         * The publish() function is how you send messages. The parameter
         * is the message object. The type of this object must agree with the type
         * given as a template parameter to the advertise<>() call, as was done
         * in the constructor above.
         */
        chatter_pub.publish(msg);

        ros::spinOnce();
        loop_rate.sleep();
        ++count;
    }

    log("Exiting from JNI call.");
    return 0;
}

JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_PsmNativeNode_shutdown
        (JNIEnv *, jobject) {
    log("Shutting down native node.");
    ros::shutdown();
    return 0;
}
