#pragma once

#include <camera/NdkCameraMetadata.h>

#include <string>

namespace android_ros {
struct CameraDescriptor
{
  std::string GetName();

  // An id identifying the camera
  uint8_t id;

  // Which way the lens if facing (back, external or front).
  acamera_metadata_enum_acamera_lens_facing lens_facing;
  // TODO intrinsics, supported resolutions, supported frame rates,
  // distortion parameters, etc.
};
}  // namespace android_ros
