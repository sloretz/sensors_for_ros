#include "sensors/accelerometer_sensor.h"

#include "log.h"

using sensors_for_ros::AccelerometerSensor;

void AccelerometerSensor::OnEvent(const ASensorEvent& event) {
  if (ASENSOR_TYPE_ACCELEROMETER != event.type) {
    LOGW("Event type was unexpected: %d", event.type);
    return;
  }
  // LOGI("vector %f, %f, %f", event.vector.x, event.vector.y, event.vector.z);
  geometry_msgs::msg::AccelStamped msg;
  // TODO(sloretz) header and frame id
  msg.accel.linear.x = event.vector.x;
  msg.accel.linear.y = event.vector.y;
  msg.accel.linear.z = event.vector.z;
  Emit(msg);
}
