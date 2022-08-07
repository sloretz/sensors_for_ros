#pragma once

namespace android_ros {
class Controller {
  public:
    Controller() = default;
    virtual ~Controller() = default;

    // Called by the GUI to draw a frame
    virtual void DrawFrame() = 0;
};
}  // namespace android_ros
