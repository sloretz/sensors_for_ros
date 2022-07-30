#include "sensors.h"

#include "jvm.h"
#include "log.h"

using android_ros::SensorDescriptor;
using android_ros::Sensors;
using android_ros::Sensor;
using android_ros::IlluminanceSensor;

SensorDescriptor::SensorDescriptor(ASensorRef _sensor_ref)
    : sensor_ref(_sensor_ref) {
  name = ASensor_getName(sensor_ref);
  type = ASensor_getType(sensor_ref);
  type_str = ASensor_getStringType(sensor_ref);
  vendor = ASensor_getVendor(sensor_ref);
  handle = ASensor_getHandle(sensor_ref);
  min_delay = ASensor_getMinDelay(sensor_ref);
  resolution = ASensor_getResolution(sensor_ref);
  // TODO(sloretz) position
}

Sensors::Sensors(ANativeActivity* activity) {
  // TODO(sloretz) Query sensors
  std::string package_name = GetPackageName(activity);
  sensor_manager_ = ASensorManager_getInstanceForPackage(package_name.c_str());

  auto descriptors = QuerySensors();
  // Create wrapping classes
  for (const SensorDescriptor & desc : descriptors) {
    if (ASENSOR_TYPE_LIGHT == desc.type) {
      sensors_.push_back(
        std::move(std::make_unique<IlluminanceSensor>(sensor_manager_, desc)));
    }
  }
}

void Sensor::Initialize() {
  shutdown_.store(false);
  queue_thread_ = std::thread(&Sensor::EventLoop, this);
}

void Sensors::Initialize() {
  for (auto & sensor : sensors_) {
    sensor->Initialize();
  }
}

void Sensor::Shutdown() {
  shutdown_.store(true);
  if (queue_thread_.joinable()) {
    // TODO(sloretz) Check looper_ isn't null
    ALooper_wake(looper_);
    queue_thread_.join();
  }
}

void Sensors::Shutdown() {
  for (auto & sensor : sensors_) {
    sensor->Shutdown();
  }
  sensors_.clear();
}

std::vector<SensorDescriptor> Sensors::QuerySensors() {
  std::vector<SensorDescriptor> descriptors;
  ASensorList sensor_list;
  int num_sensors = ASensorManager_getSensorList(sensor_manager_, &sensor_list);
  if (num_sensors < 0) {
    LOGW("Could not get sensor list: %d", num_sensors);
    return {};
  }
  LOGI("Got %d sensors", num_sensors);

  while (num_sensors > 0) {
    --num_sensors;
    ASensorRef sensor_ref = sensor_list[num_sensors];
    if (nullptr == sensor_ref) {
      LOGW("Got null sensor at position :%d", num_sensors);
      continue;
    }
    descriptors.emplace_back(sensor_ref);
    LOGI("Sensor %s name %s", descriptors.back().type_str, descriptors.back().name);
  }
  return descriptors;
}

/// Have event loop reading each sensor
void Sensor::EventLoop() {
  const int kSensorIdent = 42;
  // Looper is thread specific, so must create it here.
  looper_ = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
  ASensorEventQueue* queue = ASensorManager_createEventQueue(
      manager_, looper_, kSensorIdent, nullptr, nullptr);

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
        LOGI("Event from sensor handle %d", event.sensor);
        assert(event.sensor == descriptor_.handle);
        OnEvent(event);
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
    }
  }
  ALooper_release(looper_);
  ASensorManager_destroyEventQueue(manager_, queue);
}

void IlluminanceSensor::OnEvent(const ASensorEvent& event)
{
  if (ASENSOR_TYPE_LIGHT != event.type) {
    LOGW("Event type was unexpected: %d", event.type);
    return;
  }
  LOGI("Light level %f lx", event.light);
  event::IlluminanceChanged ice;
  ice.handle = event.sensor;
  ice.light = event.light;
  Emit(ice);
}
