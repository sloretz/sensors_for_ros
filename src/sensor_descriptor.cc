#include "sensor_descriptor.h"

using sensors_for_ros::SensorDescriptor;

SensorDescriptor::SensorDescriptor(ASensorRef _sensor_ref)
    : sensor_ref(_sensor_ref) {
  name = ASensor_getName(sensor_ref);
  type = ASensor_getType(sensor_ref);
  type_str = ASensor_getStringType(sensor_ref);
  vendor = ASensor_getVendor(sensor_ref);
  handle = ASensor_getHandle(sensor_ref);
  min_delay = ASensor_getMinDelay(sensor_ref);
  resolution = ASensor_getResolution(sensor_ref);
  // TODO(sloretz) position ???
}
