#include "camera_descriptor.h"

#include<sstream>

using android_ros::CameraDescriptor;

std::string
CameraDescriptor::GetName() {
  std::stringstream name;
  switch (lens_facing) {
  case ACAMERA_LENS_FACING_BACK:
    name << "back ";
    break;
  case ACAMERA_LENS_FACING_EXTERNAL:
    name << "external ";
    break;
  case ACAMERA_LENS_FACING_FRONT:
    name << "front ";
    break;
  default:
    break;
  }

  name << "camera (";
  name << static_cast<int>(id);
  name << ")";
  return name.str();
}
