#pragma once

#include <variant>

namespace android_ros {
namespace event {
// Event fired to indicate the ROS_DOMAIN_ID was set
struct RosDomainIdChanged {
  int32_t id;
};

using Event = std::variant<RosDomainIdChanged>;

using Listener = std::function<void(const Event&)>;

class Emitter {
 public:
  void Emit(const Event& event);
  void SetListener(Listener);

 private:
  Listener event_listener_;
};
}  // namespace event
}  // namespace android_ros