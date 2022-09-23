#include "camera_manager.h"

#include "log.h"

#include <media/NdkImage.h>

#include <memory>

using android_ros::CameraDescriptor;
using android_ros::CameraDevice;
using android_ros::CameraManager;


/// ***************** Camera manager stuff ********************
CameraManager::CameraManager() {
  native_manager_ = ACameraManager_create();

  DiscoverCameras();
}

void CameraManager::DiscoverCameras() {
  cameras_.clear();
  // Get camera IDs and put them in RAII container
  std::unique_ptr<ACameraIdList, decltype(&ACameraManager_deleteCameraIdList)> camera_ids{nullptr, nullptr};
  {
    ACameraIdList * camera_ids_temp = nullptr;
    ACameraManager_getCameraIdList(native_manager_, &camera_ids_temp);
    camera_ids = {camera_ids_temp, &ACameraManager_deleteCameraIdList};
  }
  
  for (int i = 0; i < camera_ids->numCameras; ++i) 
  {
    // Get camera's metadata in more convenient type
    CameraDescriptor cam_desc;

    cam_desc.id = camera_ids->cameraIds[i];
    std::unique_ptr<ACameraMetadata, decltype(&ACameraMetadata_free)> metadata{nullptr, nullptr};
    {
      ACameraMetadata* metadata_temp;
      ACameraManager_getCameraCharacteristics(
        native_manager_, cam_desc.id.c_str(), &metadata_temp);
      metadata = {metadata_temp, &ACameraMetadata_free};
    }
    
    int32_t num_tags = 0;
    const uint32_t* tags = nullptr;
    ACameraMetadata_getAllTags(metadata.get(), &num_tags, &tags);


    ACameraMetadata_const_entry entry = { 0 };

    auto status = ACameraMetadata_getConstEntry(metadata.get(), ACAMERA_LENS_FACING, &entry);
    if (ACAMERA_OK != status) {
      LOGW("Unable to get ACAMERA_LENS_FACING from camera %d", i);
      continue;
    }
    cam_desc.lens_facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(
            entry.data.u8[0]);

    LOGI("Looking up available stream configurations");
    // Figure out what kinds of data we can get from the camera
    ACameraMetadata_getConstEntry(metadata.get(), ACAMERA_SCALER_AVAILABLE_STREAM_CONFIGURATIONS, &entry);
    for (int i = 0; i < entry.count; i += 4)
    {
        // We are only interested in output streams, so skip input stream
        int32_t input = entry.data.i32[i + 3];
        if (input) {
            continue;
        }
    
        int32_t format = entry.data.i32[i + 0];
        if (format == AIMAGE_FORMAT_YUV_420_888) 
        {
            // This one is always supported.
            // https://developer.android.com/ndk/reference/group/media
            // #group___media_1gga9c3dace30485a0f28163a882a5d65a19aea9797f9b5db5d26a2055a43d8491890
            int32_t width = entry.data.i32[i + 1];
            int32_t height = entry.data.i32[i + 2];
            LOGI("YUV_420_888 supported w: %d h: %d", width, height);
        }
    }
    
    cameras_.push_back(cam_desc);
  }
}

CameraManager::~CameraManager() {
  if (native_manager_) {
    ACameraManager_delete(native_manager_);
  }
}

std::unique_ptr<CameraDevice>
CameraManager::OpenCamera(const CameraDescriptor & desc)
{
  return std::move(CameraDevice::OpenCamera(native_manager_, desc));
}
