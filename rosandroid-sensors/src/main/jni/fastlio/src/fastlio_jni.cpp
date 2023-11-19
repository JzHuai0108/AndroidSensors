
#include <android/log.h>

#include <sstream>
#include <ros/ros.h>
#include "std_msgs/String.h"

#include "fastlio_jni.h"
#include "fast_lio/fastlio/laserMapping.hpp"
#include "fast_csm_icp/gsm_wrap.h"

inline void log(const char *msg, ...) {
    va_list args;
    va_start(args, msg);
    __android_log_vprint(ANDROID_LOG_INFO, "Native_FastLio", msg, args);
    va_end(args);
}

inline std::string stdStringFromjString(JNIEnv *env, jstring java_string) {
    const char *tmp = env->GetStringUTFChars(java_string, NULL);
    std::string out(tmp);
    env->ReleaseStringUTFChars(java_string, tmp);
    return out;
}

JNIEXPORT jint JNICALL JNI_OnLoad(JavaVM *vm, void *reserved) {
    log("FastLio library has been loaded");
    // Return the JNI version
    return JNI_VERSION_1_6;
}

/*
 * Class:     org_ros_rosjava_tutorial_native_node_FastLioNativeNode
 * Method:    execute
 * Signature: (Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;)I
 */
JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_FastLioNativeNode_execute(
    JNIEnv *env, jobject obj, jstring rosMasterUri, jstring rosHostname, jstring rosNodeName,
    jobjectArray remappingArguments) {
  log("Native fastlio node started.");
  std::string master("__master:=" + stdStringFromjString(env, rosMasterUri));
  std::string hostname("__ip:=" + stdStringFromjString(env, rosHostname));
  std::string node_name(stdStringFromjString(env, rosNodeName));

  log(master.c_str());
  log(hostname.c_str());
  std::string nnmsg = "fastlio native nodename " + node_name;
  log(nnmsg.c_str());
  // Parse remapping arguments
  jsize len = env->GetArrayLength(remappingArguments);

  std::string ni = "fastlio_cpp";

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
  std::string pcdmap_path((char *) env->GetStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, 0), NULL));
  ros::init(argc, &argv[0], node_name.c_str());

  // Release JNI UTF characters
  for (int i = 0; i < len; i++) {
      env->ReleaseStringUTFChars((jstring) env->GetObjectArrayElement(remappingArguments, i),
                                 refs[i]);
  }
  delete []refs;
  delete []argv;

  ros::NodeHandle nh;
  std::shared_ptr<GSMWrap> gsm(new GSMWrap(nh));
  std::string fn_path = pcdmap_path.substr(0, pcdmap_path.find_last_of('/')) + "/";
  gsm->LoadMap(fn_path);
  ros::Rate rate(30);
  geometry_msgs::PoseStamped map_T_lidar;
  while (ros::ok()) {
    if (gsm->loc_status()) {
      map_T_lidar = gsm->init_pose();
      log("Global scan matcher returned initial position %f %f %f", map_T_lidar.pose.position.x, map_T_lidar.pose.position.y, map_T_lidar.pose.position.z);
      break;
    }
    ros::spinOnce();
    rate.sleep();
  }
  gsm.reset(); // remove the gsm node.
  std::vector<double> position{map_T_lidar.pose.position.x, map_T_lidar.pose.position.y, map_T_lidar.pose.position.z};
  nh.setParam("/mapping/init_world_t_lidar", position);
  std::vector<double> qxyzw{map_T_lidar.pose.orientation.x, map_T_lidar.pose.orientation.y, map_T_lidar.pose.orientation.z, map_T_lidar.pose.orientation.w};
  nh.setParam("/mapping/init_world_qxyzw_lidar", qxyzw);

  nh.setParam("/pcdmap", pcdmap_path);

  fastlio::LaserMapping node(nh);
  bool status = ros::ok();
  while (status) {
      ros::spinOnce();
      node.spinOnce();
      status = ros::ok();
      rate.sleep();
  }
  node.saveAndClose();

  log("Exiting from fastlio JNI call.");
  return 0;
}

/*
 * Class:     org_ros_rosjava_tutorial_native_node_FastLioNativeNode
 * Method:    shutdown
 * Signature: ()I
 */
JNIEXPORT jint JNICALL Java_org_ros_rosjava_1tutorial_1native_1node_FastLioNativeNode_shutdown
  (JNIEnv *, jobject) {
    log("Shutting down fastlio native node.");
    ros::shutdown();
    return 0;
}

