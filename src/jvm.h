#pragma once

#include <android/native_activity.h>

#include <string>
#include <vector>

// Functions that interact with the jvm

namespace sensors_for_ros {
std::string GetPackageName(ANativeActivity* activity);

void RequestPermission(ANativeActivity* activity, const char* permission);

bool HasPermission(ANativeActivity* activity, const char* permission);

std::string GetCacheDir(ANativeActivity* activity);

std::vector<std::string> GetNetworkInterfaces(ANativeActivity* activity);
}  // namespace sensors_for_ros
