#pragma once

#include <geometry_msgs/msg/accel_stamped.hpp>

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors/accelerometer_sensor.h"

namespace sensors_for_ros {
// Handles interface between sensor, ROS, and GUI
class AccelerometerSensorController
    : public Controller,
      public event::Emitter<event::GuiNavigateBack> {
 public:
  AccelerometerSensorController(AccelerometerSensor* sensor, RosInterface& ros);

  virtual ~AccelerometerSensorController() = default;

  void DrawFrame() override;

  std::string PrettyName() const override;

 protected:
  void OnSensorReading(const geometry_msgs::msg::AccelStamped& event);

 private:
  geometry_msgs::msg::AccelStamped last_msg_;
  AccelerometerSensor* sensor_;
  Publisher<geometry_msgs::msg::AccelStamped> publisher_;
};
}  // namespace sensors_for_ros
