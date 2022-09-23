#pragma once

#include "events.h"
#include "sensor.h"

#include <geometry_msgs/msg/accel_stamped.hpp>

namespace android_ros {
class AccelerometerSensor : public Sensor, public event::Emitter<geometry_msgs::msg::AccelStamped>
{
  public:
    using Sensor::Sensor;
    virtual ~AccelerometerSensor() = default;
  protected:
    void OnEvent(const ASensorEvent & event) override;
};
}  // namespace android_ros
