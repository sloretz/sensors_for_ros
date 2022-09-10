#include "camera_device.h"


using android_ros::CameraDevice;

/// ***************** Android C callbacks ********************

// Android camera state callback called when a camera is no longer available
void OnCameraDisconnected(void *context, ACameraDevice *device) {
  LOGI("Camera Disconnected (context pointer %p, camera device pointer %p",  context, device);
}

// Android camera state callback called when a camera is no longer available because of an error
void OnCameraError(void *context, ACameraDevice *device, int error) {
  LOGI("Camera error (context pointer %p, camera device pointer %p, error %d)",  context, device, error);
}

static const ACameraDevice_stateCallbacks kCameraStateCallbacks = {
    .context = nullptr,
    .onDisconnected = OnCameraDisconnected,
    .onError = OnCameraError,
};

/// ***************** CameraDevice ********************
std::unique_ptr<CameraDevice>
CameraDevice::OpenCamera(ACameraManager * native_manager, const CameraDescriptor & desc)
{
  const char * camera_id = desc.id.c_str();
  auto camera_device = std::unique_ptr<CameraDevice>(new CameraDevice);
  camera_device->desc_ = desc;

  auto result = ACameraManager_openCamera(
    native_manager, camera_id,
    &(camera_device->state_callbacks_),
    &(camera_device->native_device_));

  if (ACAMERA_OK != result) {
    LOGW("Failed to open camera %s, %d", camera_id, result);
    return nullptr;
  }
  LOGI("XXX I opened a camera!");
  return camera_device;
}

CameraDevice::CameraDevice() : state_callbacks_(kCameraStateCallbacks)
{
  state_callbacks_.context = this;
}

CameraDevice::~CameraDevice()
{
  LOGI("TODO CameraDevice destructor");
}
