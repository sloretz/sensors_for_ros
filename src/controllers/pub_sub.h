#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors/accelerometer_sensor.h"
#include "ros_subscriber.h"

namespace sensors_for_ros {
// Handles interface between sensor, ROS, and GUI
class PubSub
    : public Controller,
      public event::Emitter<event::GuiNavigateBack> {
 public:
  PubSub(RosInterface& ros);

  virtual ~PubSub() = default;

  void DrawFrame() override;

  std::string PrettyName() const override;

 protected:
  void OnMsgReceived(const sensors_for_ros::event::RosStringMessageReceived& event);

 private:
  rclcpp::Time now_;
  rclcpp::Time last_time_;
  double topic_hz_;
  std_msgs::msg::String last_msg_;
  RosStringSubscriber sub_;
  RosInterface& ros_;
  Publisher<std_msgs::msg::String> publisher_;
};
}  // namespace sensors_for_ros
