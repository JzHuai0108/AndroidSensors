#include <android/log.h>
#include <ros/ros.h>

#include "movebase_jni.h"
#include <move_base/move_base.h>

/*robot_state_publisher headers begin*/
#include <urdf/model.h>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

#include "robot_state_publisher/robot_state_publisher.h"
#include "robot_state_publisher/joint_state_listener.h"
/*robot_state_publisher headers end*/

#include "map_server/map_server.h"

#include "std_msgs/String.h"
#include <tf2_ros/transform_listener.h>
#include <sstream>
#include <fstream>

using namespace std;

void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "Native_MoveBase", msg, args);
    va_end(args);
}

inline string stdStringFromjString(JNIEnv *env, jstring java_string) {
    const char *tmp = env->GetStringUTFChars(java_string, NULL);
    string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

bool running;

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    log("Movebase library has been loaded");
    // Return the JNI version
    return JNI_VERSION_1_6;
}

JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_MoveBaseNativeNode_execute(
        JNIEnv *env, jobject obj, jstring rosMasterUri, jstring rosHostname, jstring rosNodeName,
        jobjectArray remappingArguments) {
    log("Native movebase node started.");
    running = true;

    string master("__master:=" + stdStringFromjString(env, rosMasterUri));
    string hostname("__ip:=" + stdStringFromjString(env, rosHostname));
    string node_name(stdStringFromjString(env, rosNodeName));

    log(master.c_str());
    log(hostname.c_str());
    string nnmsg = "movebase native nodename " + node_name;
    log(nnmsg.c_str());

    // Parse remapping arguments
    log("Before getting size");
    jsize len = env->GetArrayLength(remappingArguments);
    log("After reading size");

    std::string ni = "movebase_cpp";

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
    std::string urdf_filename((char *) env->GetStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, 0), NULL));
    std::string map_yaml((char *) env->GetStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, 1), NULL));

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

    ros::NodeHandle nh;
    nh.setParam("/robot_state_publisher/publish_frequency", 50.0);
    nh.setParam("/robot_state_publisher/tf_prefix", "");

    // startRobotStatePublisher
    std::ifstream in(urdf_filename, std::ios::in | std::ios::binary);
    if (!in.good()) {
        ROS_ERROR("Failed to open urdf file %s.", urdf_filename.c_str());
        return -1;
    }
    in.seekg(0, std::ios::end);
    std::string urdf_xml;
    urdf_xml.resize(in.tellg());
    in.seekg(0, std::ios::beg);
    in.read(&urdf_xml[0], urdf_xml.size());
    in.close();
    nh.setParam("robot_description", urdf_xml);

    urdf::Model model;
    if (!model.initParam("robot_description")) {
        ROS_ERROR("Failed to init model from robot_description");
        return -1;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromUrdfModel(model, tree)) {
        ROS_ERROR("Failed to extract kdl tree from xml robot description");
        return -1;
    }

    MimicMap mimic;

    for(std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++) {
        if(i->second->mimic) {
        mimic.insert(make_pair(i->first, i->second->mimic));
        }
    }

    robot_state_publisher::JointStateListener state_publisher(tree, mimic, model);

    // start map server
    std::ifstream map_in(map_yaml, std::ios::in | std::ios::binary);
    if (!map_in.good()) {
        ROS_ERROR("Failed to open map yaml %s.", map_yaml.c_str());
        return -1;
    } else {
        ROS_INFO("Map yaml %s is good", map_yaml.c_str());
    }
    double res = 0.0;
    MapServer ms(map_yaml, res);

    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("mb_chatter", 1000);
    ros::Rate loop_rate(10);

    nh.setParam("/move_base/base_local_planner", "dwa_local_planner/DWAPlannerROS");
    nh.setParam("/move_base/base_global_planner", "navfn/NavfnROS");
    nh.setParam("/move_base/clearing_rotation_allowed", true);
    nh.setParam("/move_base/conservative_reset/reset_distance", 3.0);

    tf2_ros::Buffer buffer(ros::Duration(10));
    tf2_ros::TransformListener tf(buffer);
    log("movebase starting.");
    move_base::MoveBase move_base(buffer);
    log("movebase running.");

    int count = 0;
    while (ros::ok()) {
        /**
         * This is a message object. You stuff it with data, and then publish it.
         */
        std_msgs::String msg;
        std::stringstream ss;
        ss << "movebase hello world " << count;
        msg.data = ss.str();
        // ROS_INFO("%s", msg.data.c_str());

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

JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_MoveBaseNativeNode_shutdown
        (JNIEnv *, jobject) {
    log("Shutting down native node.");
    ros::shutdown();
    running = false;
    return 0;
}
