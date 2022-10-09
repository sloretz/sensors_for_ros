#include "sensors/magnetometer_sensor.h"

#include "log.h"

using android_ros::MagnetometerSensor;

void MagnetometerSensor::OnEvent(const ASensorEvent& event) {
  if (ASENSOR_TYPE_MAGNETIC_FIELD != event.type) {
    LOGW("Event type was unexpected: %d", event.type);
    return;
  }
  // LOGI("vector %f, %f, %f", event.vector.x, event.vector.y, event.vector.z);
  sensor_msgs::msg::MagneticField msg;
  // TODO(sloretz) header and frame id
  msg.magnetic_field.x = event.magnetic.x / kMicroTeslaPerTesla;
  msg.magnetic_field.y = event.magnetic.y / kMicroTeslaPerTesla;
  msg.magnetic_field.z = event.magnetic.z / kMicroTeslaPerTesla;
  Emit(msg);
}
