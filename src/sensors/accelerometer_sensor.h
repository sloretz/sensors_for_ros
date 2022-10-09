#pragma once

#include <geometry_msgs/msg/accel_stamped.hpp>

#include "events.h"
#include "sensor.h"

namespace android_ros {
class AccelerometerSensor
    : public Sensor,
      public event::Emitter<geometry_msgs::msg::AccelStamped> {
 public:
  using Sensor::Sensor;
  virtual ~AccelerometerSensor() = default;

 protected:
  void OnEvent(const ASensorEvent& event) override;
};
}  // namespace android_ros
