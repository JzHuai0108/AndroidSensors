#include "als_mcl_jni.h"
#include <android/log.h>
#include <ros/ros.h>
#include <als_ros/MCL.h>

#include "std_msgs/String.h"
#include <sstream>

using namespace std;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "Native_Amcl", msg, args);
    va_end(args);
}

inline string stdStringFromjString(JNIEnv *env, jstring java_string) {
    const char *tmp = env->GetStringUTFChars(java_string, NULL);
    string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

int loop_count_ = 0;

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    log("ALS MCL library has been loaded");
    return JNI_VERSION_1_6;
}

JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_AlsMclNativeNode_execute
(JNIEnv *env, jobject obj, jstring rosMasterUri, jstring rosHostname, jstring rosNodeName,
        jobjectArray remappingArguments) {
    log("Native als mcl node started.");
    string master("__master:=" + stdStringFromjString(env, rosMasterUri));
    string hostname("__ip:=" + stdStringFromjString(env, rosHostname));
    string node_name(stdStringFromjString(env, rosNodeName));

    string nnmsg = "als mcl native nodename " + node_name;
    log(nnmsg.c_str());
    // Parse remapping arguments
    jsize len = env->GetArrayLength(remappingArguments);

    std::string ni = "als_mcl_cpp";

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
    std::string classifier_asset_dir((char *) env->GetStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, 0), NULL));

    ros::init(argc, &argv[0], node_name.c_str());

    // Release JNI UTF characters
    for (int i = 0; i < len; i++) {
        env->ReleaseStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, i),
                                   refs[i]);
    }
    delete []refs;
    delete []argv;

    ros::NodeHandle nh;
    nh.setParam("/als_mcl/mae_classifier_dir", classifier_asset_dir);
    log("mcl mae_classifier_dir %s", classifier_asset_dir.c_str());
    als_ros::MCL mcl;
    double localizationHz = mcl.getLocalizationHz();
    ros::Rate loopRate(localizationHz);
    int count = 0;
    // for debug purposes, we use ros::spinOnce(). Otherwise,ros::spin() without while loop is enough.
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("als_mcl_chatter", 100);
    while (ros::ok()) {
        std_msgs::String msg;
        std::stringstream ss;
        ss << "als mcl hello world " << count;
        msg.data = ss.str();
        chatter_pub.publish(msg);

        ros::spinOnce();
        mcl.updateParticlesByMotionModel();
        mcl.setCanUpdateScan(false);
        mcl.calculateLikelihoodsByMeasurementModel();
        mcl.calculateLikelihoodsByDecisionModel();
        mcl.calculateGLSampledPosesLikelihood();
        mcl.calculateAMCLRandomParticlesRate();
        mcl.calculateEffectiveSampleSize();
        mcl.estimatePose();
        mcl.resampleParticles();
        mcl.publishROSMessages();
        mcl.broadcastTF();
        // mcl.plotLikelihoodMap();
        mcl.setCanUpdateScan(true);
        if (count < 5) {
            mcl.printResult();
        }
        loopRate.sleep();
        ++count;
    }

    log("Exiting from als mcl JNI call.");
    return 0;
}

JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_AlsMclNativeNode_shutdown
  (JNIEnv *, jobject) {
  log("Shutting down als mcl native node.");
  ros::shutdown();
  return 0;
}

