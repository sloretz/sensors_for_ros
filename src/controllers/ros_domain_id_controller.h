#pragma once

#include "controller.h"
#include "events.h"

namespace android_ros {

// There should only be one of these in existance.
constexpr const char* kRosDomainIdControllerId = "ros_domain_id_controller";

class RosDomainIdController : public Controller,
                              public event::Emitter<event::RosDomainIdChanged> {
 public:
  RosDomainIdController() : Controller(kRosDomainIdControllerId) {}
  virtual ~RosDomainIdController(){};

  // Called by the GUI to draw a frame
  void DrawFrame() override;

  std::string PrettyName() const override { return "ROS Domain ID"; }
};
}  // namespace android_ros
