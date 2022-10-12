#pragma once

#include <geometry_msgs/msg/twist_stamped.hpp>

#include "events.h"
#include "sensor.h"

namespace sensors_for_ros {
class GyroscopeSensor
    : public Sensor,
      public event::Emitter<geometry_msgs::msg::TwistStamped> {
 public:
  using Sensor::Sensor;
  virtual ~GyroscopeSensor() = default;

 protected:
  void OnEvent(const ASensorEvent& event) override;
};
}  // namespace sensors_for_ros
