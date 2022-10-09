#pragma once

#include <EGL/egl.h>
#include <GLES/gl.h>
#include <android/input.h>
#include <android/native_activity.h>

#include <atomic>
#include <future>
#include <mutex>
#include <thread>

#include "controller.h"
#include "events.h"

namespace android_ros {
class GUI {
 public:
  GUI();
  ~GUI();

  void Start(ANativeActivity* activity, ANativeWindow* window);
  void Stop();

  void SetInputQueue(AInputQueue* queue);
  void RemoveInputQueue();

  void SetController(Controller* controller);

 private:
  void InitializeDearImGui(ANativeWindow* window);
  void TerminateDearImGui();
  bool InitializeDisplay(ANativeWindow* window);
  void TerminateDisplay();
  void DrawFrame();
  void ShowROSDomainIdPicker();
  void DrawingLoop(ANativeWindow* window,
                   std::promise<void> promise_first_frame);
  void CheckInput();

  EGLDisplay display_;
  EGLSurface surface_;
  EGLContext context_;
  int32_t width_;
  int32_t height_;

  std::thread draw_thread_;
  std::atomic<bool> exit_loop_;

  std::mutex iqueue_mtx_;
  AInputQueue* iqueue_ = nullptr;

  ANativeActivity* activity_;

  Controller* active_controller = nullptr;
};
}  // namespace android_ros
