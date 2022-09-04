#pragma once

#include <vector>

#include <camera/NdkCameraManager.h>

#include "camera_descriptor.h"

namespace android_ros {
class CameraManager
{
public:
  CameraManager();
  ~CameraManager();

  const std::vector<CameraDescriptor> & GetCameras() { return cameras_; }

private:
  void DiscoverCameras();


  ACameraManager* native_manager_ = nullptr;
  std::vector<CameraDescriptor> cameras_;
};
}  // namespace android_ros
