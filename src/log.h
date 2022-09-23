#pragma once

#include <android/log.h>

#define LOGI(...) \
  ((void)__android_log_print(ANDROID_LOG_INFO, "android_ros", __VA_ARGS__))
#define LOGW(...) \
((void)__android_log_print(ANDROID_LOG_WARN, "android_ros", __VA_ARGS__))
