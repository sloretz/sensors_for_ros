#include "events.h"

namespace android_ros {
namespace event {
void Emitter::Emit(const Event& event) { event_listener_(event); }
void Emitter::SetListener(std::function<void(const Event&)> listener) {
  event_listener_ = listener;
}

}  // namespace event
}  // namespace android_ros
