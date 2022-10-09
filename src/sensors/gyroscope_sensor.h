#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "events.h"
#include "sensor.h"

namespace android_ros {
class GyroscopeSensor
    : public Sensor,
      public event::Emitter<geometry_msgs::msg::TwistStamped> {
 public:
  using Sensor::Sensor;
  virtual ~GyroscopeSensor() = default;

 protected:
  void OnEvent(const ASensorEvent& event) override;
};
}  // namespace android_ros
