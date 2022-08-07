#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors.h"

#include <sensor_msgs/msg/illuminance.hpp>

namespace android_ros {
// Handles interface between sensor, ROS, and GUI
class IlluminanceSensorController : public Controller, public event::Emitter<event::GuiNavigateBack> {
  public:
    // TODO reference to GUI? Or maybe GUI has reference to this!
    IlluminanceSensorController(
      IlluminanceSensor* sensor,
      Publisher<sensor_msgs::msg::Illuminance> publisher);

    virtual ~IlluminanceSensorController() = default;

    void DrawFrame() override;

  protected:
    void
    OnIlluminanceChanged(const event::IlluminanceChanged& event);

  private:
    // TODO do I even need this pointer?
    Sensor* sensor_;
    Publisher<sensor_msgs::msg::Illuminance> publisher_;
};
}  // namespace android_ros
