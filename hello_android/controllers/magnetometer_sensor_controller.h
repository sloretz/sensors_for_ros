#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors/magnetometer_sensor.h"

#include <sensor_msgs/msg/magnetic_field.hpp>

namespace android_ros {
// Handles interface between sensor, ROS, and GUI
class MagnetometerSensorController : public Controller, public event::Emitter<event::GuiNavigateBack> {
  public:
    MagnetometerSensorController(
      MagnetometerSensor* sensor,
      RosInterface& ros);

    virtual ~MagnetometerSensorController() = default;

    void DrawFrame() override;

  protected:
    void
    OnSensorReading(const sensor_msgs::msg::MagneticField& event);

  private:
    sensor_msgs::msg::MagneticField last_msg_;
    MagnetometerSensor* sensor_;
    Publisher<sensor_msgs::msg::MagneticField> publisher_;
};
}  // namespace android_ros

