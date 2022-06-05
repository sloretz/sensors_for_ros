#include "sensors.h"

#include "jvm.h"
#include "log.h"

using android_ros::SensorDescriptor;
using android_ros::Sensors;

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

  sensors_ = QuerySensors();
}

void Sensors::Initialize(rclcpp::Context::SharedPtr context,
                         rclcpp::Node::SharedPtr node) {
  context_ = context;
  node_ = node;

  shutdown_.store(false);
  queue_thread_ = std::thread(&Sensors::EventLoop, this);
}

void Sensors::Shutdown() {
  shutdown_.store(true);
  if (queue_thread_.joinable()) {
    // TODO(sloretz) Check looper_ isn't null
    ALooper_wake(looper_);
    queue_thread_.join();
  }
  sensors_.clear();
  context_.reset();
  node_.reset();
}

std::vector<SensorDescriptor> Sensors::QuerySensors() {
  ASensorList sensor_list;
  int num_sensors = ASensorManager_getSensorList(sensor_manager_, &sensor_list);
  if (num_sensors < 0) {
    LOGW("Could not get sensor list: %d", num_sensors);
    return {};
  }
  LOGI("Got %d sensors", num_sensors);

  std::vector<SensorDescriptor> found_sensors;
  found_sensors.reserve(num_sensors);
  while (num_sensors > 0) {
    --num_sensors;
    ASensorRef sensor_ref = sensor_list[num_sensors];
    if (nullptr == sensor_ref) {
      LOGW("Got null sensor at position :%d", num_sensors);
      continue;
    }
    found_sensors.push_back(SensorDescriptor(sensor_ref));
    LOGI("Sensor %s name %s", found_sensors.back().type_str,
         found_sensors.back().name);
  }
  return found_sensors;
}

const std::vector<SensorDescriptor>& Sensors::GetSensors() { return sensors_; }

void Sensors::EventLoop() {
  LOGI("Entering sensor event loop");
  const int kSensorIdent = 42;
  // Looper is thread specific, so must create it here.
  looper_ = ALooper_prepare(ALOOPER_PREPARE_ALLOW_NON_CALLBACKS);
  ASensorEventQueue* queue = ASensorManager_createEventQueue(
      sensor_manager_, looper_, kSensorIdent, nullptr, nullptr);

  // Debugging, just enable all sensors
  for (const SensorDescriptor& sensor : sensors_) {
    if (ASENSOR_TYPE_LIGHT == sensor.type) {
      LOGI("Enabling light sensor %s", sensor.name);
      ASensorEventQueue_enableSensor(queue, sensor.sensor_ref);
    }
  }

  // Read all pending events.
  int ident;
  int events;
  void* data;
  LOGI("About to go into sensor looper loop");
  while (not shutdown_.load()) {
    ident = ALooper_pollAll(-1, nullptr, &events, &data);
    if (kSensorIdent == ident) {
      LOGI("Looper awoke, with sensor event");
      ASensorEvent event;
      while (ASensorEventQueue_getEvents(queue, &event, 1) > 0) {
        LOGI("Event from sensor handle %d", event.sensor);
        if (ASENSOR_TYPE_LIGHT == event.type) {
          LOGI("Light level %f", event.light);
        } else {
          LOGI("Event type was unexpected: %d", event.type);
        }
        // LOGI("accelerometer: x=%f y=%f z=%f",
        //      event.acceleration.x, event.acceleration.y,
        //      event.acceleration.z);
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
  ASensorManager_destroyEventQueue(sensor_manager_, queue);
}
