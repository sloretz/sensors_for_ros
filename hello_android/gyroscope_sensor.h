#pragma once

#include "events.h"
#include "sensor.h"

// #include <sensor_msgs/msg/illuminance.hpp>

namespace android_ros {
class GyroscopeSensor : public Sensor //, public event::Emitter<sensor_msgs::msg::Illuminance>
{
  public:
    using Sensor::Sensor;
    virtual ~GyroscopeSensor() = default;
  protected:
    void OnEvent(const ASensorEvent & event) override;
};
}  // namespace android_ros
