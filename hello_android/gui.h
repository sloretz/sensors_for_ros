#pragma once

#include <atomic>
#include <future>
#include <mutex>
#include <thread>

#include <EGL/egl.h>
#include <GLES/gl.h>

#include <android/input.h>

namespace android_ros {
class GUI {
 public:
  GUI();
  ~GUI();

  void Start(ANativeWindow* window);
  void Stop();

  void SetInputQueue(AInputQueue* queue);
  void RemoveInputQueue();


 private:
  void InitializeDearImGui(ANativeWindow* window);
  void TerminateDearImGui();
  bool InitializeDisplay(ANativeWindow* window);
  void TerminateDisplay();
  void DrawFrame();
  void DrawingLoop(ANativeWindow* window, std::promise<void> promise_first_frame);
  void CheckInput();

  EGLDisplay display_;
  EGLSurface surface_;
  EGLContext context_;
  int32_t width_;
  int32_t height_;

  std::thread draw_thread_;
  std::atomic<bool> exit_loop_;

  std::mutex iqueue_mtx_;
  AInputQueue * iqueue_ = nullptr;
};
}  // namespace android_ros