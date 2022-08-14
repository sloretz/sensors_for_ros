#include "gyroscope_sensor.h"

#include "log.h"

using android_ros::GyroscopeSensor;

void GyroscopeSensor::OnEvent(const ASensorEvent& event)
{
  if (ASENSOR_TYPE_GYROSCOPE != event.type) {
    LOGW("Event type was unexpected: %d", event.type);
    return;
  }
  LOGI("vector %f, %f, %f", event.vector.x, event.vector.y, event.vector.z);
  // sensor_msgs::msg::gyroscope msg;
  // msg.gyroscope = event.light;
  // Emit(msg);
}


