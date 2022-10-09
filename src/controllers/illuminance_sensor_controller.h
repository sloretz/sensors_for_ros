#pragma once

#include <sensor_msgs/msg/illuminance.hpp>

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors/illuminance_sensor.h"

namespace android_ros {
// Handles interface between sensor, ROS, and GUI
class IlluminanceSensorController
    : public Controller,
      public event::Emitter<event::GuiNavigateBack> {
 public:
  // TODO reference to GUI? Or maybe GUI has reference to this!
  IlluminanceSensorController(IlluminanceSensor* sensor, RosInterface& ros);

  virtual ~IlluminanceSensorController() = default;

  void DrawFrame() override;

  std::string PrettyName() const override;

 protected:
  void OnIlluminanceChanged(const sensor_msgs::msg::Illuminance& event);

 private:
  sensor_msgs::msg::Illuminance last_msg_;
  IlluminanceSensor* sensor_;
  Publisher<sensor_msgs::msg::Illuminance> publisher_;
};
}  // namespace android_ros
