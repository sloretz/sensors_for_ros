#include "ros_interface.h"

#include "log.h"

using android_ros::ROSInterface;

ROSInterface::ROSInterface() { context_ = std::make_shared<rclcpp::Context>(); }

void ROSInterface::Initialize(size_t ros_domain_id) {
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(ros_domain_id);
  init_options.shutdown_on_signal = false;
  context_->init(0, nullptr, init_options);

  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  node_ = std::make_shared<rclcpp::Node>("android_ros", node_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_shared<rclcpp::executors::StaticSingleThreadedExecutor>(
      executor_options);
  executor_->add_node(node_);

  executor_thread_ = std::thread(&rclcpp::Executor::spin, executor_.get());
}

void ROSInterface::Shutdown() {
  context_->shutdown("ROSInterface asked to Shutdown");
  executor_thread_.join();
  node_.reset();
  executor_.reset();
}

bool ROSInterface::Initialized() { return context_->is_valid(); }