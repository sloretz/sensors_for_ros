#pragma once

#include <thread>
#include <variant>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/illuminance.hpp>
#include <sensor_msgs/msg/range.hpp>

#include "events.h"
#include "log.h"

namespace android_ros {
class RosInterface {
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

/// Interface to A ROS publisher
// I want this class to insulate the Sensor interface from ROS going up or down
// or publishers being created etc when settings change
// This is to be used by the sensor interface to publish data
template <typename MsgT>
class Publisher {
 public:
  Publisher(const RosInterface& ros) : ros_(ros) {}

  // Moves ok
  Publisher(Publisher&& other) = default;
  Publisher& operator=(Publisher&& other) = default;
  // No copies please
  Publisher(const Publisher& other) = delete;
  Publisher& operator=(const Publisher& other) = delete;

  // TODO(sloretz) this class needs to be notified when RosInterface turns ROS on or off

  void Enable() {
    LOGI("Asked to enable publisher");
    auto node = ros_.get_node();
    publisher_ = node->template create_publisher<MsgT>(topic_, qos_);
  }

  void Disable() {
    LOGI("Asked to disable publisher");
    publisher_.reset();
  }

  // Big messages to avoid copies
  void Publish(const typename MsgT::SharedPtr& msg) const {
    if (publisher_) {
      publisher_->publish(msg);
    }
  }

  // Little messages to avoid heap allocation
  void Publish(const MsgT& msg) const {
    LOGI("asked to publish message");
    if (publisher_) {
      LOGI("did publish message");
      publisher_->publish(msg);
    }
  }

 private:
  const RosInterface &ros_;
  std::string topic_ = "default_topic";
  rclcpp::QoS qos_ = rclcpp::QoS(1);
  typename rclcpp::Publisher<MsgT>::SharedPtr publisher_;
};
}  // namespace android_ros
