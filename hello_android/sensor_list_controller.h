#pragma once

#include "controller.h"
#include "events.h"
#include "log.h"
#include "sensors.h"

namespace android_ros {
class SensorListController
  : public Controller,
    public event::Emitter<event::GuiNavigateBack>,
    public event::Emitter<event::GuiNavigateToSensor>
{
  public:
    using event::Emitter<event::GuiNavigateBack>::SetListener;
    using event::Emitter<event::GuiNavigateToSensor>::SetListener;
    using event::Emitter<event::GuiNavigateBack>::Emit;
    using event::Emitter<event::GuiNavigateToSensor>::Emit;

    SensorListController(android_ros::Sensors& sensors);

    virtual ~SensorListController() = default;

    void DrawFrame() override;

  private:
    android_ros::Sensors& sensors_;
};
}  // namespace android_ros

