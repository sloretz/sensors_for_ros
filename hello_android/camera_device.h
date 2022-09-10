#pragma once

#include "camera_descriptor.h"
#include "log.h"

#include <camera/NdkCameraManager.h>

#include <memory>

namespace android_ros {
class CameraDevice {
public:
  static
  std::unique_ptr<CameraDevice>
  OpenCamera(ACameraManager * native_manager, const CameraDescriptor & desc);

  ~CameraDevice();

  const CameraDescriptor & GetDescriptor() const { return desc_; }

private:
  CameraDevice();

  CameraDescriptor desc_;
  ACameraDevice * native_device_ = nullptr;
  ACameraDevice_stateCallbacks state_callbacks_;
};
}  // namespace android_ros
