#pragma once

#include <string>

#include <android/native_activity.h>

// Functions that interact with the jvm

namespace android_ros {
std::string GetPackageName(ANativeActivity* activity);

void RequestPermission(ANativeActivity* activity, const char * permission);

bool HasPermission(ANativeActivity* activity, const char * permission);
}
