#include "camera_descriptor.h"

#include <sstream>

using android_ros::CameraDescriptor;

std::string CameraDescriptor::GetName() const {
  std::stringstream name;
  switch (lens_facing) {
    case ACAMERA_LENS_FACING_BACK:
      name << "Back ";
      break;
    case ACAMERA_LENS_FACING_EXTERNAL:
      name << "External ";
      break;
    case ACAMERA_LENS_FACING_FRONT:
      name << "Front ";
      break;
    default:
      break;
  }

  name << "camera (" << id << ")";
  return name.str();
}
