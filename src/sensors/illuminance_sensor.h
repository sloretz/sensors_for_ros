#pragma once

#include <sensor_msgs/msg/illuminance.hpp>

#include "events.h"
#include "sensor.h"

namespace sensors_for_ros {
class IlluminanceSensor : public Sensor,
                          public event::Emitter<sensor_msgs::msg::Illuminance> {
 public:
  using Sensor::Sensor;
  virtual ~IlluminanceSensor() = default;

 protected:
  void OnEvent(const ASensorEvent& event) override;
};
}  // namespace sensors_for_ros
