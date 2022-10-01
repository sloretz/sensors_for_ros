#include <cassert>

#include "sensor.h"

#include "log.h"

using android_ros::Sensor;

/// Have event loop reading each sensor
void Sensor::EventLoop() {
  const int kSensorIdent = 42;
  // Looper is thread specific, so must create it here.
  looper_ = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
  ASensorEventQueue* queue = ASensorManager_createEventQueue(
      manager_, looper_, kSensorIdent, nullptr, nullptr);

  // Ask for addional info events to have sensor position
  if (0 != ASensorEventQueue_requestAdditionalInfoEvents(queue, true)) {
    LOGW("Couldn't enable additional info events");
  }

  ASensorEventQueue_enableSensor(queue, descriptor_.sensor_ref);

  // Read all pending events.
  int ident;
  int events;
  void* data;
  while (not shutdown_.load()) {
    ident = ALooper_pollAll(-1, nullptr, &events, &data);
    if (kSensorIdent == ident) {
      ASensorEvent event;
      while (ASensorEventQueue_getEvents(queue, &event, 1) > 0) {
        // LOGI("Event from sensor handle %d", event.sensor);
        assert(event.sensor == descriptor_.handle);
        if (ASENSOR_TYPE_ADDITIONAL_INFO == event.type) {
          LOGI("Additional info type: %d", event.additional_info.type);
          if (ASENSOR_ADDITIONAL_INFO_SENSOR_PLACEMENT == event.additional_info.type) {
            LOGI("Got position!");
            // TODO store this position somewhere
          }
        } else {
          OnEvent(event);
        }
      }
    } else if (ALOOPER_POLL_WAKE == ident) {
      LOGI("Looper was manually woken up");
    } else if (ALOOPER_POLL_CALLBACK == ident) {
      LOGI("Callbacks were called? I don't care");
    } else if (ALOOPER_POLL_TIMEOUT == ident) {
      LOGW("Loop with infinite timeout timed out");
    } else if (ALOOPER_POLL_ERROR == ident) {
      LOGW("Some error occured, shutting down");
      shutdown_.store(true);
    } else {
      LOGI("Uknown ident %d", ident);
    }
  }
  ASensorEventQueue_disableSensor(queue, descriptor_.sensor_ref);
  ASensorManager_destroyEventQueue(manager_, queue);
  ALooper_release(looper_);
}

void Sensor::Initialize() {
  shutdown_.store(false);
  queue_thread_ = std::thread(&Sensor::EventLoop, this);
}

void Sensor::Shutdown() {
  shutdown_.store(true);
  if (queue_thread_.joinable()) {
    // TODO(sloretz) Check looper_ isn't null
    ALooper_wake(looper_);
    queue_thread_.join();
  }
}
