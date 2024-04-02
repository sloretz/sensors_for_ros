#pragma once

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "events.h" 
#include "ros_interface.h"
#include <atomic>
#include <thread>
#include "android/looper.h"


namespace sensors_for_ros {

class RosStringSubscriber : 
public event::Emitter<event::RosStringMessageReceived>
{
public:
    RosStringSubscriber(RosInterface& ros);
    std::string get_topic() const;
private:
    void messageCallback(const std_msgs::msg::String::SharedPtr msg);

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    event::Emitter<event::RosStringMessageReceived> stringMessageEmitter_;
    std::string topic_name_;
};

} // namespace sensors_for_ros
