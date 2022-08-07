#include "ros_interface.h"

#include "log.h"

using android_ros::RosInterface;

RosInterface::RosInterface() {}

void RosInterface::Initialize(size_t ros_domain_id) {
  rclcpp::InitOptions init_options;
  init_options.set_domain_id(ros_domain_id);
  init_options.shutdown_on_signal = false;
  context_ = std::make_shared<rclcpp::Context>(); 
  context_->init(0, nullptr, init_options);

  rclcpp::NodeOptions node_options;
  node_options.context(context_);
  node_ = std::make_shared<rclcpp::Node>("android_ros", node_options);

  rclcpp::ExecutorOptions executor_options;
  executor_options.context = context_;
  executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>(
      executor_options);
  executor_->add_node(node_);

  executor_thread_ = std::thread(&rclcpp::Executor::spin, executor_.get());

  NotifyInitChanged();
}

void RosInterface::Shutdown() {
  context_->shutdown("RosInterface asked to Shutdown");
  NotifyInitChanged();
  executor_thread_.join();
  node_.reset();
  executor_.reset();
  context_.reset();
}

bool RosInterface::Initialized() const { return context_ && context_->is_valid(); }

rclcpp::Context::SharedPtr RosInterface::get_context() const {
  return context_;
}

rclcpp::Node::SharedPtr RosInterface::get_node() const { return node_; }

void RosInterface::AddObserver(std::function<void(void)> init_or_shutdown) {
  observers_.push_back(init_or_shutdown);
}

void RosInterface::NotifyInitChanged() {
  for (auto & observer : observers_) {
    observer();
  }
  // Still want observations? ask for them again
  observers_.clear();
}
