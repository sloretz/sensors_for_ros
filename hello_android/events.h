#pragma once

#include <variant>

namespace android_ros {
namespace event {
// Event fired to indicate the ROS_DOMAIN_ID was set
struct RosDomainIdChanged {
  int32_t id;
};

struct SensorEvent {
  // Sensor handle (ASensorEvent::sensor) uniquely identifying the sensor
  int handle;
};

// Indicates an illuminance sensor got a new reading
struct IlluminanceChanged : SensorEvent {
  // SI unit: lux (lx)
  float light;
};

// Indicates a controller wants to exit back to the previous page
struct GuiNavigateBack {
};

// Indicates a controller wants to display a specific sensor
struct GuiNavigateToSensor {
  // Sensor handle (ASensorEvent::sensor) uniquely identifying the sensor
  int handle;
};

template <typename EventType>
using Listener = std::function<void(const EventType&)>;

template <typename EventType>
class Emitter {
 public:
  void Emit(const EventType& event) {
    if (event_listener_) {
      event_listener_(event);
    }
  }

  void SetListener(Listener<EventType> listener) {
    event_listener_ = listener;
  }

 private:
  Listener<EventType> event_listener_;
};
}  // namespace event
}  // namespace android_ros
