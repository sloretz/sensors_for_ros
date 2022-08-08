#pragma once

#include "events.h"
#include "ros_interface.h"

#include <android/looper.h>
#include <android/native_activity.h>
#include <android/sensor.h>

#include <variant>

#include <sensor_msgs/msg/illuminance.hpp>

namespace android_ros {


// TODO document who uses this
struct SensorDescriptor {
  explicit SensorDescriptor(ASensorRef _sensor_ref);
  SensorDescriptor(const SensorDescriptor& other) = default;
  ~SensorDescriptor() = default;

  const char * PrettyType() const {
    switch (type) {
      case ASENSOR_TYPE_LIGHT:
        return "Light";
        break;
      default:
        return type_str;
    }
  }

  ASensorRef sensor_ref;
  const char* name;
  const char* type_str;
  const char* vendor;
  int type;
  int handle;
  int min_delay;
  float resolution;
};


class Sensor {
  public:
    Sensor(ASensorManager* manager, SensorDescriptor desc) : manager_(manager), descriptor_(desc) {}
    virtual ~Sensor() = default;

  void Initialize();
  void Shutdown();

  const SensorDescriptor& Descriptor() { return descriptor_; }

  protected:
    void EventLoop();
    
    virtual void OnEvent(const ASensorEvent & event) = 0;

  private:
    const SensorDescriptor descriptor_;

  ASensorManager* manager_ = nullptr;
  std::atomic<bool> shutdown_;
  std::thread queue_thread_;
  ALooper* looper_ = nullptr;
};

class IlluminanceSensor : public Sensor, public event::Emitter<sensor_msgs::msg::Illuminance>
{
  public:
    using Sensor::Sensor;
    virtual ~IlluminanceSensor() = default;
  protected:
    void OnEvent(const ASensorEvent & event) override;
};


class Sensors {
 public:
  Sensors(ANativeActivity* activity);
  ~Sensors() = default;

  void Initialize();
  void Shutdown();

  const std::vector<std::unique_ptr<Sensor>> &
  GetSensors() {return sensors_;};

 private:
  std::vector<SensorDescriptor> QuerySensors();

  void EventLoop();

  ASensorManager* sensor_manager_ = nullptr;

  std::vector<std::unique_ptr<Sensor>> sensors_;
};
}  // namespace android_ros
