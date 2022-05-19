#pragma once

#include <EGL/egl.h>
#include <GLES/gl.h>

namespace android_ros {
class GUI {
 public:
  GUI();
  ~GUI();

  bool InitializeDisplay(ANativeWindow* window);
  void TerminateDisplay();

 private:
  void DrawFrame();

  EGLDisplay display_;
  EGLSurface surface_;
  EGLContext context_;
  int32_t width_;
  int32_t height_;
};
}  // namespace android_ros