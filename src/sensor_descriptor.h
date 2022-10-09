#pragma once
#include <android/looper.h>
#include <android/sensor.h>

namespace android_ros {
// TODO document who uses this
struct SensorDescriptor {
  explicit SensorDescriptor(ASensorRef _sensor_ref);
  SensorDescriptor(const SensorDescriptor& other) = default;
  ~SensorDescriptor() = default;

  const char* PrettyType() const {
    switch (type) {
      case ASENSOR_TYPE_LIGHT:
        return "Light";
        break;
      case ASENSOR_TYPE_GYROSCOPE:
        return "Gyroscope";
        break;
      case ASENSOR_TYPE_ACCELEROMETER:
        return "Accelerometer";
        break;
      case ASENSOR_TYPE_PRESSURE:
        return "Barometer";
        break;
      case ASENSOR_TYPE_MAGNETIC_FIELD:
        return "Magnetometer";
        break;
      default:
        return type_str;
    }
  }

  ASensorRef sensor_ref;
  const char* name;
  const char* type_str;
  const char* vendor;
  int type;
  int handle;
  int min_delay;
  float resolution;
};
}  // namespace android_ros
