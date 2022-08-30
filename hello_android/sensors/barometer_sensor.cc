#include "sensors/barometer_sensor.h"

#include "log.h"

using android_ros::BarometerSensor;

void BarometerSensor::OnEvent(const ASensorEvent& event)
{
  if (ASENSOR_TYPE_PRESSURE != event.type) {
    LOGW("Event type was unexpected: %d", event.type);
    return;
  }
  sensor_msgs::msg::FluidPressure msg;
  // TODO(sloretz) header and frame id
  // Convert millibar to pascals
  const int kPascalsPerMillibar = 100;
  msg.fluid_pressure = event.pressure * kPascalsPerMillibar;
  Emit(msg);
}
