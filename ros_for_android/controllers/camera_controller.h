#pragma once

#include <memory>

#include "camera_device.h"
#include "controller.h"
#include "events.h"
#include "ros_interface.h"

namespace android_ros {
class CameraController : public Controller, public event::Emitter<event::GuiNavigateBack> {
  public:
    CameraController(
      std::unique_ptr<CameraDevice> device,
      RosInterface& ros);
    virtual ~CameraController();

    // Called by the GUI to draw a frame
    void DrawFrame() override;

    std::string PrettyName() const override;

  protected:
    void
    OnImage(const std::pair<CameraInfo::UniquePtr, Image::UniquePtr> & info_image);

  private:
    std::unique_ptr<CameraDevice> device_;
    Publisher<sensor_msgs::msg::CameraInfo> info_pub_;
    Publisher<sensor_msgs::msg::Image> image_pub_;
};
}  // namespace android_ros
