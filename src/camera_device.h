#pragma once

#include <camera/NdkCameraManager.h>
#include <media/NdkImage.h>
#include <media/NdkImageReader.h>

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>
#include <tuple>
#include <utility>

#include "camera_descriptor.h"
#include "events.h"
#include "log.h"

namespace sensors_for_ros {
struct AImageDeleter {
  void operator()(AImage* image) { AImage_delete(image); }
};

using sensor_msgs::msg::CameraInfo;
using sensor_msgs::msg::Image;

class CameraDevice : public event::Emitter<
                         std::pair<CameraInfo::UniquePtr, Image::UniquePtr>> {
 public:
  static std::unique_ptr<CameraDevice> OpenCamera(
      ACameraManager* native_manager, const CameraDescriptor& desc);

  ~CameraDevice();

  const CameraDescriptor& GetDescriptor() const { return desc_; }

  // Internal
  void OnImage(std::unique_ptr<AImage, AImageDeleter> image);

  std::tuple<int, int> Resolution() const { return {width_, height_}; }

 private:
  CameraDevice();

  // TODO Open supported formats we queried with the descriptor
  int width_ = 640;
  int height_ = 480;

  // Get images from camera and convert them to sensor_msgs/msgs/Image
  void ProcessImages();

  CameraDescriptor desc_;
  ACameraDevice* native_device_ = nullptr;
  ACameraDevice_stateCallbacks state_callbacks_;

  AImageReader* reader_ = nullptr;
  AImageReader_ImageListener reader_callbacks_;

  ACameraOutputTarget* camera_output_target_ = nullptr;
  ACaptureSessionOutput* capture_session_output_ = nullptr;

  ACaptureSessionOutputContainer* output_container_ = nullptr;

  ACaptureRequest* capture_request_ = nullptr;

  ACameraCaptureSession* capture_session_ = nullptr;

  std::mutex mutex_;
  std::unique_ptr<AImage, AImageDeleter> image_ = nullptr;
  std::atomic<bool> shutdown_;
  std::condition_variable wake_cv_;
  std::thread thread_;
};
}  // namespace sensors_for_ros
