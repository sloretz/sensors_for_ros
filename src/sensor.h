#pragma once

#include <android/sensor.h>

#include <atomic>
#include <thread>

#include "sensor_descriptor.h"

namespace android_ros {
class Sensor {
 public:
  Sensor(ASensorManager* manager, SensorDescriptor desc)
      : manager_(manager), descriptor_(desc) {}
  virtual ~Sensor() = default;

  void Initialize();
  void Shutdown();

  const SensorDescriptor& Descriptor() { return descriptor_; }

 protected:
  void EventLoop();

  virtual void OnEvent(const ASensorEvent& event) = 0;

 private:
  const SensorDescriptor descriptor_;

  ASensorManager* manager_ = nullptr;
  std::atomic<bool> shutdown_;
  std::thread queue_thread_;
  ALooper* looper_ = nullptr;
};
}  // namespace android_ros
