#pragma once

#include "events.h"

#include <android/looper.h>
#include <android/native_activity.h>
#include <android/sensor.h>

#include <variant>

#include <rclcpp/context.hpp>
#include <rclcpp/node.hpp>

namespace android_ros {

struct SensorDescriptor {
  SensorDescriptor(ASensorRef _sensor_ref);
  ~SensorDescriptor() = default;

  ASensorRef sensor_ref;
  const char* name;
  const char* type_str;
  const char* vendor;
  int type;
  int handle;
  int min_delay;
  float resolution;

  bool enabled = false;
};

class Sensors {
 public:
  Sensors(ANativeActivity* activity);
  ~Sensors() = default;

  void Initialize(rclcpp::Context::SharedPtr context,
                  rclcpp::Node::SharedPtr node);
  void Shutdown();

  const std::vector<SensorDescriptor>& GetSensors();

 private:
  std::vector<SensorDescriptor> QuerySensors();

  void EventLoop();

  ASensorManager* sensor_manager_ = nullptr;
  rclcpp::Context::SharedPtr context_;
  rclcpp::Node::SharedPtr node_;

  std::vector<SensorDescriptor> sensors_;

  std::atomic<bool> shutdown_;
  std::thread queue_thread_;
  ALooper* looper_ = nullptr;
};
}  // namespace android_ros
