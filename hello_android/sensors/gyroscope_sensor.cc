#include "sensors/gyroscope_sensor.h"

#include "log.h"

using android_ros::GyroscopeSensor;

void GyroscopeSensor::OnEvent(const ASensorEvent& event)
{
  if (ASENSOR_TYPE_GYROSCOPE != event.type) {
    LOGW("Event type was unexpected: %d", event.type);
    return;
  }
  // LOGI("vector %f, %f, %f", event.vector.x, event.vector.y, event.vector.z);
  geometry_msgs::msg::TwistStamped msg;
  // TODO(sloretz) header and frame id
  msg.twist.angular.x = event.vector.x;
  msg.twist.angular.y = event.vector.y;
  msg.twist.angular.z = event.vector.z;
  Emit(msg);
}


