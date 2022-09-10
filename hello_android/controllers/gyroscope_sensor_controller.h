#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors/gyroscope_sensor.h"

#include <geometry_msgs/msg/twist_stamped.hpp>

namespace android_ros {
// Handles interface between sensor, ROS, and GUI
class GyroscopeSensorController : public Controller, public event::Emitter<event::GuiNavigateBack> {
  public:
    GyroscopeSensorController(
      GyroscopeSensor* sensor,
      RosInterface& ros);

    virtual ~GyroscopeSensorController() = default;

    void DrawFrame() override;

    std::string PrettyName() const override;

  protected:
    void
    OnGyroReading(const geometry_msgs::msg::TwistStamped& event);

  private:
    geometry_msgs::msg::TwistStamped last_msg_;
    GyroscopeSensor* sensor_;
    Publisher<geometry_msgs::msg::TwistStamped> publisher_;
};
}  // namespace android_ros

