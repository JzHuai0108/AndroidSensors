#include "glposesampler_jni.h"

#include <android/log.h>
#include <ros/ros.h>
#include <als_ros/GLPoseSampler.h>

#include "std_msgs/String.h"
#include <sstream>

using namespace std;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "Native_GLPoseSampler", msg, args);
    va_end(args);
}

inline string stdStringFromjString(JNIEnv *env, jstring java_string) {
    const char *tmp = env->GetStringUTFChars(java_string, NULL);
    string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    log("GLPoseSampler library has been loaded");
    return JNI_VERSION_1_6;
}


JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_GLPoseSamplerNativeNode_execute
(JNIEnv *env, jobject obj, jstring rosMasterUri, jstring rosHostname, jstring rosNodeName,
        jobjectArray remappingArguments) {
    log("Native glposesampler node started.");
    string master("__master:=" + stdStringFromjString(env, rosMasterUri));
    string hostname("__ip:=" + stdStringFromjString(env, rosHostname));
    string node_name(stdStringFromjString(env, rosNodeName));

    string nnmsg = "glposesampler native nodename " + node_name;
    log(nnmsg.c_str());
    // Parse remapping arguments
    jsize len = env->GetArrayLength(remappingArguments);

    std::string ni = "glposesampler_cpp";

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

    ros::init(argc, &argv[0], node_name.c_str());

    // Release JNI UTF characters
    for (int i = 0; i < len; i++) {
        env->ReleaseStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, i),
                                   refs[i]);
    }
    delete []refs;
    delete []argv;

    ros::NodeHandle nh;
    als_ros::GLPoseSampler sampler;
    ros::Rate loopRate(10);
    int count = 0;
    // for debug purposes, we use ros::spinOnce(). Otherwise,ros::spin() without while loop is enough.
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("glposesampler_chatter", 100);
    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "glposesampler hello world " << count;
        msg.data = ss.str();
        chatter_pub.publish(msg);

        ros::spinOnce();
        loopRate.sleep();
        ++count;
    }

    log("Exiting from glposesampler JNI call.");
    return 0;
}

JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_GLPoseSamplerNativeNode_shutdown
  (JNIEnv *, jobject) {
  log("Shutting down glposesampler native node.");
  ros::shutdown();
  return 0;
}

