#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors/barometer_sensor.h"

#include <sensor_msgs/msg/fluid_pressure.hpp>

namespace android_ros {
// Handles interface between sensor, ROS, and GUI
class BarometerSensorController : public Controller, public event::Emitter<event::GuiNavigateBack> {
  public:
    BarometerSensorController(
      BarometerSensor* sensor,
      RosInterface& ros);

    virtual ~BarometerSensorController() = default;

    void DrawFrame() override;

    std::string PrettyName() const override;

  protected:
    void
    OnSensorReading(const sensor_msgs::msg::FluidPressure& event);

  private:
    sensor_msgs::msg::FluidPressure last_msg_;
    BarometerSensor* sensor_;
    Publisher<sensor_msgs::msg::FluidPressure> publisher_;
};
}  // namespace android_ros
