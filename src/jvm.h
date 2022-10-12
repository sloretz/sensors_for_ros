#pragma once

#include <android/native_activity.h>

#include <string>

// Functions that interact with the jvm

namespace sensors_for_ros {
std::string GetPackageName(ANativeActivity* activity);

void RequestPermission(ANativeActivity* activity, const char* permission);

bool HasPermission(ANativeActivity* activity, const char* permission);
}  // namespace sensors_for_ros
