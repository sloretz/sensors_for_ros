#pragma once

#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "events.h"

namespace android_ros {
class RosInterface : public android_ros::event::Emitter {
 public:
  RosInterface();
  ~RosInterface() = default;

  void Initialize(size_t ros_domain_id);
  void Shutdown();

  bool Initialized() const;

  rclcpp::Context::SharedPtr get_context() const;
  rclcpp::Node::SharedPtr get_node() const;

 private:
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Executor::SharedPtr executor_;

  std::thread executor_thread_;
};
}  // namespace android_ros
