#pragma once

#include <camera/NdkCameraManager.h>

#include <vector>

#include "camera_descriptor.h"
#include "camera_device.h"

namespace android_ros {
class CameraManager {
 public:
  CameraManager();
  ~CameraManager();

  bool HasCameras() const { return !cameras_.empty(); }

  const std::vector<CameraDescriptor>& GetCameras() { return cameras_; }

  std::unique_ptr<CameraDevice> OpenCamera(const CameraDescriptor& desc) const;

 private:
  void DiscoverCameras();

  ACameraManager* native_manager_ = nullptr;
  std::vector<CameraDescriptor> cameras_;
};
}  // namespace android_ros
