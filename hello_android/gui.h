#pragma once

#include <atomic>
#include <thread>

#include <EGL/egl.h>
#include <GLES/gl.h>

namespace android_ros {
class GUI {
 public:
  GUI();
  ~GUI();

  bool Start(ANativeWindow* window);
  void Stop();


 private:
  bool InitializeDisplay(ANativeWindow* window);
  void TerminateDisplay();
  void DrawFrame();
  void DrawingLoop();

  EGLDisplay display_;
  EGLSurface surface_;
  EGLContext context_;
  int32_t width_;
  int32_t height_;

  std::thread draw_thread_;
  std::atomic<bool> exit_loop_;
};
}  // namespace android_ros