MY_LOCAL_PATH := $(call my-dir)
LOCAL_PATH=$(MY_LOCAL_PATH)

# include roscpp
include /media/jhuai/docker/roscpp_android_ndk/ros_android/output/roscpp_android_ndk/jni/Android.mk
LOCAL_PATH=$(MY_LOCAL_PATH)

include $(CLEAR_VARS)
LOCAL_PATH      :=$(MY_LOCAL_PATH)/psm
LOCAL_MODULE    := psm_jni
LOCAL_CFLAGS    := -std=c++11 -g -O0 -DDEBUG
LOCAL_CPPFLAGS  := -isystem
LOCAL_CPP_FEATURES := exceptions
LOCAL_SRC_FILES := $(LOCAL_PATH)/src/psm_jni.cpp $(LOCAL_PATH)/src/psm_singleton.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_LDFLAGS := -Wl,--exclude-libs,libgcc.a -Wl,--exclude-libs,libgnustl_shared.so
LOCAL_LDLIBS := -landroid -llog
LOCAL_STATIC_LIBRARIES := roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)


include $(CLEAR_VARS)
LOCAL_PATH      :=$(MY_LOCAL_PATH)/movebase
LOCAL_MODULE    := movebase_jni
LOCAL_CFLAGS    := -std=c++11 -g -O0 -DDEBUG
LOCAL_CPPFLAGS  := -isystem
LOCAL_CPP_FEATURES := exceptions
LOCAL_SRC_FILES := $(LOCAL_PATH)/src/movebase_jni.cpp
LOCAL_C_INCLUDES += $(LOCAL_PATH)/include
LOCAL_LDFLAGS := -Wl,--exclude-libs,libgcc.a -Wl,--exclude-libs,libgnustl_shared.so
LOCAL_LDLIBS := -landroid -llog
LOCAL_STATIC_LIBRARIES := roscpp_android_ndk
include $(BUILD_SHARED_LIBRARY)
