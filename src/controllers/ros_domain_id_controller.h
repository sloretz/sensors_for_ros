#pragma once

#include "controller.h"
#include "events.h"

#include <android/native_activity.h>

#include <string>
#include <vector>

namespace sensors_for_ros {

// There should only be one of these in existance.
constexpr const char* kRosDomainIdControllerId = "ros_domain_id_controller";

class RosDomainIdController : public Controller,
                              public event::Emitter<event::RosDomainIdChanged> {
 public:
  RosDomainIdController(ANativeActivity* activity);
  virtual ~RosDomainIdController(){};

  // Called by the GUI to draw a frame
  void DrawFrame() override;

  std::string PrettyName() const override { return "ROS Domain ID"; }

 private:
  std::string preferred_interface_;
  std::vector<std::string> network_interfaces_;
};
}  // namespace sensors_for_ros
