#include "color_listener.hpp"

ColorListener::ColorListener(const char * name, const rclcpp::NodeOptions & options)
  : Node(name, options)
{
    color_sub_ = rclcpp::create_subscription<std_msgs::msg::ColorRGBA>(
        *this,
        "color",
        // TODO configurable QoS
        rclcpp::QoS(1),
        std::bind(&ColorListener::on_new_color, this, std::placeholders::_1));
}

void
ColorListener::on_new_color(const std_msgs::msg::ColorRGBA & color)
{
  RCLCPP_INFO(get_logger(), "Got new color message (%f, %f, %f, %f)",
      color.r, color.g, color.b, color.a);
  std::lock_guard<std::mutex> guard(mutex_);
  color_ = color;
}

std_msgs::msg::ColorRGBA
ColorListener::color() const
{
  std::lock_guard<std::mutex> guard(mutex_);
  return color_;
}
