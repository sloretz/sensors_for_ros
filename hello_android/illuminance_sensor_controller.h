#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors.h"

#include <sensor_msgs/msg/illuminance.hpp>

namespace android_ros {
// Handles interface between sensor, ROS, and GUI
class IlluminanceSensorController : public Controller
{
  public:
    // TODO reference to GUI? Or maybe GUI has reference to this!
    IlluminanceSensorController(
      IlluminanceSensor* sensor,
      Publisher<sensor_msgs::msg::Illuminance> publisher)
      : sensor_(sensor), publisher_(std::move(publisher))
    {
      sensor->SetListener(
        std::bind(&IlluminanceSensorController::OnIlluminanceChanged, this, std::placeholders::_1));

      // TODO allow publisher to be enabled/disabled from GUI
      publisher_.Enable();

      // TODO allow GUI to change topic and QoS
    }

    virtual ~IlluminanceSensorController() = default;

  protected:
    void
    OnIlluminanceChanged(const event::IlluminanceChanged& event) {
      auto msg = sensor_msgs::msg::Illuminance();
      // TODO(sloretz) time and frame id
      msg.illuminance = event.light;
      msg.variance = 0.0;
      publisher_.Publish(msg);
      LOGI("Publishing ROS message %lf lx", msg.illuminance);
    }

  private:
    // TODO do I even need this pointer?
    Sensor* sensor_;
    Publisher<sensor_msgs::msg::Illuminance> publisher_;

};
}  // namespace android_ros
