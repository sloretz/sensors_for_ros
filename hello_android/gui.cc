#include "gui.h"

#include "log.h"

#include <memory>

using android_ros::GUI;

GUI::GUI() {
// TODO(sloretz) raise if EGL display can't be initialized
// InitializeDisplay();
// TODO(sloretz) Start thread to draw window
}

GUI::~GUI(){
	TerminateDisplay();
}

bool GUI::InitializeDisplay(ANativeWindow * window) {
  // Copied from
  // https://developer.android.com/ndk/samples/sample_na
  // initialize OpenGL ES and EGL

  /*
   * Here specify the attributes of the desired configuration.
   * Below, we select an EGLConfig with at least 8 bits per color
   * component compatible with on-screen windows
   */
  const EGLint attribs[] = {EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                            EGL_BLUE_SIZE,    8,
                            EGL_GREEN_SIZE,   8,
                            EGL_RED_SIZE,     8,
                            EGL_NONE};
  EGLint w, h, format;
  EGLint numConfigs;
  EGLConfig config = nullptr;
  EGLSurface surface;
  EGLContext context;

  EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

  eglInitialize(display, nullptr, nullptr);

  /* Here, the application chooses the configuration it desires.
   * find the best match if possible, otherwise use the very first one
   */
  eglChooseConfig(display, attribs, nullptr, 0, &numConfigs);
  std::unique_ptr<EGLConfig[]> supportedConfigs(new EGLConfig[numConfigs]);
  assert(supportedConfigs);
  eglChooseConfig(display, attribs, supportedConfigs.get(), numConfigs,
                  &numConfigs);
  assert(numConfigs);
  auto i = 0;
  for (; i < numConfigs; i++) {
    auto& cfg = supportedConfigs[i];
    EGLint r, g, b, d;
    if (eglGetConfigAttrib(display, cfg, EGL_RED_SIZE, &r) &&
        eglGetConfigAttrib(display, cfg, EGL_GREEN_SIZE, &g) &&
        eglGetConfigAttrib(display, cfg, EGL_BLUE_SIZE, &b) &&
        eglGetConfigAttrib(display, cfg, EGL_DEPTH_SIZE, &d) && r == 8 &&
        g == 8 && b == 8 && d == 0) {
      config = supportedConfigs[i];
      break;
    }
  }
  if (i == numConfigs) {
    config = supportedConfigs[0];
  }

  if (config == nullptr) {
    LOGW("Unable to initialize EGLConfig");
    return false;
  }

  /* EGL_NATIVE_VISUAL_ID is an attribute of the EGLConfig that is
   * guaranteed to be accepted by ANativeWindow_setBuffersGeometry().
   * As soon as we picked a EGLConfig, we can safely reconfigure the
   * ANativeWindow buffers to match, using EGL_NATIVE_VISUAL_ID. */
  eglGetConfigAttrib(display, config, EGL_NATIVE_VISUAL_ID, &format);
  surface =
      eglCreateWindowSurface(display, config, window, nullptr);
  context = eglCreateContext(display, config, nullptr, nullptr);

  if (eglMakeCurrent(display, surface, surface, context) == EGL_FALSE) {
    LOGW("Unable to eglMakeCurrent");
    return false;
  }

  eglQuerySurface(display, surface, EGL_WIDTH, &w);
  eglQuerySurface(display, surface, EGL_HEIGHT, &h);

  display_ = display;
  context_ = context;
  surface_ = surface;
  width_ = w;
  height_ = h;

  // Check openGL on the system
  auto opengl_info = {GL_VENDOR, GL_RENDERER, GL_VERSION, GL_EXTENSIONS};
  for (auto name : opengl_info) {
    auto info = glGetString(name);
    LOGI("OpenGL Info: %s", info);
  }
  // Initialize GL state.
  glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_FASTEST);
  glEnable(GL_CULL_FACE);
  glShadeModel(GL_SMOOTH);
  glDisable(GL_DEPTH_TEST);

  return true;
}

void GUI::DrawFrame() {
  float red = 1.0f;
  float green = 0.0f;
  float blue = 0.0f;

  // Just fill the screen with a color.
  glClearColor(red, green, blue, 1);
  glClear(GL_COLOR_BUFFER_BIT);

  eglSwapBuffers(display_, surface_);
}

void GUI::TerminateDisplay() {
  // Copied from
  // https://developer.android.com/ndk/samples/sample_na
  if (display_ != EGL_NO_DISPLAY) {
    eglMakeCurrent(display_, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (context_ != EGL_NO_CONTEXT) {
      eglDestroyContext(display_, context_);
    }
    if (surface_ != EGL_NO_SURFACE) {
      eglDestroySurface(display_, surface_);
    }
    eglTerminate(display_);
  }
  display_ = EGL_NO_DISPLAY;
  context_ = EGL_NO_CONTEXT;
  surface_ = EGL_NO_SURFACE;
}
