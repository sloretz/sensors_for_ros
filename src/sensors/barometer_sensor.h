#pragma once

#include <sensor_msgs/msg/fluid_pressure.hpp>

#include "events.h"
#include "sensor.h"

namespace sensors_for_ros {
class BarometerSensor : public Sensor,
                        public event::Emitter<sensor_msgs::msg::FluidPressure> {
 public:
  using Sensor::Sensor;
  virtual ~BarometerSensor() = default;

 protected:
  void OnEvent(const ASensorEvent& event) override;
};
}  // namespace sensors_for_ros
