#pragma once

#include "events.h"
#include "sensor.h"

#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace android_ros {
class BarometerSensor : public Sensor, public event::Emitter<sensor_msgs::msg::FluidPressure>
{
  public:
    using Sensor::Sensor;
    virtual ~BarometerSensor() = default;
  protected:
    void OnEvent(const ASensorEvent & event) override;
};
}  // namespace android_ros
