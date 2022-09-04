#include "camera_manager.h"

#include <memory>

using android_ros::CameraDescriptor;
using android_ros::CameraManager;

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
    // Get camera's metadata in an RAII type
    const char* id = camera_ids->cameraIds[i];
    std::unique_ptr<ACameraMetadata, decltype(&ACameraMetadata_free)> metadata{nullptr, nullptr};
    {
      ACameraMetadata* metadata_temp;
      ACameraManager_getCameraCharacteristics(native_manager_, id, &metadata_temp);
      metadata = {metadata_temp, &ACameraMetadata_free};
    }
    
    int32_t num_tags = 0;
    const uint32_t* tags = nullptr;
    ACameraMetadata_getAllTags(metadata.get(), &num_tags, &tags);

    CameraDescriptor cam_desc;
    cam_desc.id = i;

    for (int t = 0; t < num_tags; ++t)
    {
        if (ACAMERA_LENS_FACING == tags[t]) {
            ACameraMetadata_const_entry lens_info = { 0 };
            ACameraMetadata_getConstEntry(metadata.get(), tags[t], &lens_info);

            cam_desc.lens_facing = static_cast<acamera_metadata_enum_android_lens_facing_t>(
                    lens_info.data.u8[0]);
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
