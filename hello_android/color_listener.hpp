#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>

class ColorListener : public rclcpp::Node
{
  public:
    ColorListener(const char * name)
      : ColorListener(name, rclcpp::NodeOptions())
    {
    }

    ColorListener(const char * name, const rclcpp::NodeOptions & options);

    virtual ~ColorListener() = default;

    /// Thread-safe accessor for current color
    std_msgs::msg::ColorRGBA color() const; 

  private:
    void on_new_color(const std_msgs::msg::ColorRGBA & color);

    rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr color_sub_;
    std_msgs::msg::ColorRGBA color_;
    mutable std::mutex mutex_;
};
