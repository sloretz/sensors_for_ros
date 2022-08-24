#pragma once

#include "controller.h"
#include "events.h"

namespace android_ros {
class RosDomainIdController : public Controller, public event::Emitter<event::RosDomainIdChanged> {
  public:
    RosDomainIdController() = default;
    virtual ~RosDomainIdController() {};

    // Called by the GUI to draw a frame
    void DrawFrame() override;
};
}  // namespace android_ros
