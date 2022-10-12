#pragma once

#include <android/native_activity.h>
#include <android/sensor.h>

#include <vector>

#include "events.h"
#include "ros_interface.h"
#include "sensor.h"
#include "sensor_descriptor.h"

namespace sensors_for_ros {
class Sensors {
 public:
  Sensors(ANativeActivity* activity);
  ~Sensors() = default;

  void Initialize();
  void Shutdown();

  const std::vector<std::unique_ptr<Sensor>>& GetSensors() { return sensors_; };

 private:
  std::vector<SensorDescriptor> QuerySensors();

  void EventLoop();

  ASensorManager* sensor_manager_ = nullptr;

  std::vector<std::unique_ptr<Sensor>> sensors_;
};
}  // namespace sensors_for_ros
