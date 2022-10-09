#pragma once

#include <sensor_msgs/msg/magnetic_field.hpp>

#include "events.h"
#include "sensor.h"

namespace android_ros {
const float kMicroTeslaPerTesla = 1000000;

class MagnetometerSensor
    : public Sensor,
      public event::Emitter<sensor_msgs::msg::MagneticField> {
 public:
  using Sensor::Sensor;
  virtual ~MagnetometerSensor() = default;

 protected:
  void OnEvent(const ASensorEvent& event) override;
};
}  // namespace android_ros
