#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "accelerometer_sensor.h"

#include <geometry_msgs/msg/accel_stamped.hpp>

namespace android_ros {
// Handles interface between sensor, ROS, and GUI
class AccelerometerSensorController : public Controller, public event::Emitter<event::GuiNavigateBack> {
  public:
    AccelerometerSensorController(
      AccelerometerSensor* sensor,
      Publisher<geometry_msgs::msg::AccelStamped> publisher);

    virtual ~AccelerometerSensorController() = default;

    void DrawFrame() override;

  protected:
    void
    OnSensorReading(const geometry_msgs::msg::AccelStamped& event);

  private:
    geometry_msgs::msg::AccelStamped last_msg_;
    AccelerometerSensor* sensor_;
    Publisher<geometry_msgs::msg::AccelStamped> publisher_;
};
}  // namespace android_ros