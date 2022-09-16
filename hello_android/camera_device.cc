#include "camera_device.h"


using android_ros::CameraDevice;

/// ***************** Android OpenCamera callbacks ********************

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

/// ***************** Android AImageReader callbacks ********************

void OnImage(void* context, AImageReader* reader)
{
    AImage * cimage = nullptr;
    auto status = AImageReader_acquireNextImage(reader, &cimage);

    // TODO Check status here ...

    std::unique_ptr<AImage, android_ros::AImageDeleter> image(cimage);

    auto * camera_device = static_cast<android_ros::CameraDevice *>(context);
    camera_device->OnImage(std::move(image));
}

static const AImageReader_ImageListener kImageListenerCallbacks = {
  .context = nullptr,
  .onImageAvailable = OnImage,
};

/// ***************** Android capture session state callbacks ********************
// TODO implement these -they're still copy/pasted
static void onSessionActive(void* context, ACameraCaptureSession *session)
{}

static void onSessionReady(void* context, ACameraCaptureSession *session)
{}

static void onSessionClosed(void* context, ACameraCaptureSession *session)
{}

static ACameraCaptureSession_stateCallbacks sessionStateCallbacks {
        .context = nullptr,
        .onActive = onSessionActive,
        .onReady = onSessionReady,
        .onClosed = onSessionClosed
};

/// ***************** Android capture callbacks ********************
// TODO implement these -they're still copy/pasted
void onCaptureFailed(void* context, ACameraCaptureSession* session,
                     ACaptureRequest* request, ACameraCaptureFailure* failure)
{}

void onCaptureSequenceCompleted(void* context, ACameraCaptureSession* session,
                                int sequenceId, int64_t frameNumber)
{}

void onCaptureSequenceAborted(void* context, ACameraCaptureSession* session,
                              int sequenceId)
{}

void onCaptureCompleted (
        void* context, ACameraCaptureSession* session,
        ACaptureRequest* request, const ACameraMetadata* result)
{}

static ACameraCaptureSession_captureCallbacks captureCallbacks {
        .context = nullptr,
        .onCaptureStarted = nullptr,
        .onCaptureProgressed = nullptr,
        .onCaptureCompleted = onCaptureCompleted,
        .onCaptureFailed = onCaptureFailed,
        .onCaptureSequenceCompleted = onCaptureSequenceCompleted,
        .onCaptureSequenceAborted = onCaptureSequenceAborted,
        .onCaptureBufferLost = nullptr,
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

  // Open image reader to get camera data
  constexpr int max_simultaneous_images = 1;
  media_status_t status = AImageReader_new(
    camera_device->width_, camera_device->height_, AIMAGE_FORMAT_YUV_420_888,
                   max_simultaneous_images, &(camera_device->reader_));

  // TODO handle errors
  //if (status != AMEDIA_OK)
      // Handle errors here

  AImageReader_setImageListener(
    camera_device->reader_,
    &(camera_device->reader_callbacks_));

  ACaptureSessionOutputContainer_create(&(camera_device->output_container_));

  ACameraDevice_createCaptureRequest(
   camera_device->native_device_, TEMPLATE_RECORD,
   &(camera_device->capture_request_));

  ANativeWindow * native_window;
  AImageReader_getWindow(camera_device->reader_, &native_window);
  ANativeWindow_acquire(native_window);
  ACameraOutputTarget_create(native_window, &(camera_device->camera_output_target_));
  ACaptureRequest_addTarget(camera_device->capture_request_, camera_device->camera_output_target_);
  ACaptureSessionOutput_create(native_window, &(camera_device->capture_session_output_));
  ACaptureSessionOutputContainer_add(camera_device->output_container_, camera_device->capture_session_output_);

  ACameraDevice_createCaptureSession(
    camera_device->native_device_,
    camera_device->output_container_,
    &sessionStateCallbacks,  // TODO
    &(camera_device->capture_session_));

  // Start Recording
  ACameraCaptureSession_setRepeatingRequest(
    camera_device->capture_session_,
    &captureCallbacks,  // TODO
    1,
    &(camera_device->capture_request_),
    nullptr);

  return camera_device;
}

CameraDevice::CameraDevice() :
  state_callbacks_(kCameraStateCallbacks),
  reader_callbacks_(kImageListenerCallbacks)
{
  state_callbacks_.context = this;
  reader_callbacks_.context = this;


  shutdown_.store(false);
  thread_ = std::thread(&CameraDevice::ProcessImages, this);
}

void CameraDevice::ProcessImages()
{
  while(!shutdown_.load()) {
    std::unique_ptr<AImage, AImageDeleter> image;
    {
      // Wait for next image, or shutdown
      std::unique_lock guard(mutex_);
      wake_cv_.wait(guard, [&image, this]{
        image = std::move(image_);
        image_.reset();
        return nullptr != image.get() || shutdown_.load();
      });
    }
    if (nullptr != image.get()) {
      auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
      // Plane 0: Y
      // Plane 1: U (Cb)
      // Plane 2: V (Cr)
      // U/V planes guaranteed to have same row and pixel stride

      // Y plane guaranteed stride of 1
      constexpr int32_t y_pixel_stride = 1;
      const int32_t y_row_stride = width_;

      int32_t uv_pixel_stride;
      int32_t uv_row_stride;

      if (AMEDIA_OK != AImage_getPlanePixelStride(image.get(), 1, &uv_pixel_stride)) {
        LOGW("Unable to get U/V plane pixel stride");
        continue;
      }
      if (AMEDIA_OK != AImage_getPlaneRowStride(image.get(), 1, &uv_row_stride)) {
        LOGW("Unable to get U/V plane row stride");
        continue;
      }

      uint8_t * y_data = nullptr;
      uint8_t * u_data = nullptr;
      uint8_t * v_data = nullptr;
      int y_len = -1;
      int u_len = -1;
      int v_len = -1;

      if (AMEDIA_OK != AImage_getPlaneData(image.get(), 0, &y_data, &y_len)) {
        LOGW("Unable to get Y plane data");
        continue;
      }
      if (AMEDIA_OK != AImage_getPlaneData(image.get(), 1, &u_data, &u_len)) {
        LOGW("Unable to get U plane data");
        continue;
      }
      if (AMEDIA_OK != AImage_getPlaneData(image.get(), 2, &v_data, &v_len)) {
        LOGW("Unable to get V plane data");
        continue;
      }

      image_msg->width = width_;
      image_msg->height = height_;
      image_msg->encoding = "rgb8";
      image_msg->step = width_ * 3;
      image_msg->data.resize(image_msg->step * image_msg->height);

      // YUV420 to RGB888
      // https://blog.minhazav.dev/how-to-convert-yuv-420-sp-android.media.Image-to-Bitmap-or-jpeg
      size_t byte = 0;
      for (int h = 0; h < height_; ++h) {
        // U/V are subsampled
        const int uvh = h / 2;
        for (int w = 0; w < width_; ++w) {
          // U/V are subsampled
          const int uvw = w / 2;
          const size_t y_idx = h * y_row_stride + w * y_pixel_stride;
          const size_t uv_idx = uvh * uv_row_stride + uvw * uv_pixel_stride;
          // LOGI("y_idx %lu uv_idx %lu y_len %d, u_len %d, v_len %d", y_idx, uv_idx, y_len, u_len, v_len);
          // continue; // XXX

          const uint8_t y = y_data[y_idx];
          const uint8_t u = u_data[uv_idx];
          const uint8_t v = v_data[uv_idx];

          int r = y + (1.370705 * (v - 128));
          int g = y - (0.698001 * (v - 128)) - (0.337633 * (u - 128));;
          int b = y + (1.732446 * (u - 128));

          if (r < 0) {
            r = 0;
          } else if (r > 255) {
            r = 255;
          }
          if (g < 0) {
            g = 0;
          } else if (g > 255) {
            g = 255;
          }
          if (b < 0) {
            b = 0;
          } else if (b > 255) {
            b = 255;
          }
          image_msg->data[byte++] = r;
          image_msg->data[byte++] = g;
          image_msg->data[byte++] = b;
        }
      }

      // TODO Emit data for publisher
      LOGI("Processed image?");
      Emit({std::make_unique<CameraInfo>(), std::move(image_msg)});
    }
  }
  LOGI("Camera device ProcessImages shutting down");
}

CameraDevice::~CameraDevice()
{
  // Shut down processing thread
  shutdown_.store(true);
  if (thread_.joinable()) {
    wake_cv_.notify_one();
    thread_.join();
  }
  if (capture_session_) {
    ACameraCaptureSession_stopRepeating(capture_session_);
    ACameraCaptureSession_close(capture_session_);
  }

  if (output_container_) {
    ACaptureSessionOutputContainer_free(output_container_);
  }

  if (capture_session_output_) {
    ACaptureSessionOutput_free(capture_session_output_);
  }

  if (capture_request_) {
    ACaptureRequest_free(capture_request_);
  }

  if (reader_) {
    AImageReader_delete(reader_);
  }
  if (native_device_) {
    ACameraDevice_close(native_device_);
  }
}

void CameraDevice::OnImage(std::unique_ptr<AImage, AImageDeleter> image)
{
  // Hand off image data to processing thread
  std::unique_lock lock(mutex_);
  image_ = std::move(image);
  wake_cv_.notify_one();
}
