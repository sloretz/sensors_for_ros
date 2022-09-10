#pragma once

#include <string>

namespace android_ros {
class Controller {
  public:
    Controller(const std::string & unique_id) : unique_id_(unique_id) {}
    virtual ~Controller() = default;

    // Called by the GUI to draw a frame
    virtual void DrawFrame() = 0;

    // Called by list controller to display this in a list
    virtual std::string PrettyName() const = 0;

    // Called by list controller to uniquely identify this in a system
    const char * UniqueId() const { return unique_id_.c_str(); }

  private:
    const std::string unique_id_;
};
}  // namespace android_ros
