#include "ros_subscriber.h"

using namespace sensors_for_ros;

RosStringSubscriber::RosStringSubscriber(RosInterface& ros) 
  {
    
    if (ros.Initialized()){
        topic_name_ = "to_android";
        auto node_ = ros.get_node();
        subscription_ = node_->create_subscription<std_msgs::msg::String>(
        topic_name_, 10,
        [this](const std_msgs::msg::String::SharedPtr msg) { this->messageCallback(msg); });
        }
}

void RosStringSubscriber::messageCallback(const std_msgs::msg::String::SharedPtr msg) {
    // Create and emit an event every time a message is received
    Emit(event::RosStringMessageReceived{*msg});
}

std::string RosStringSubscriber::get_topic() const {
    return topic_name_;
}


