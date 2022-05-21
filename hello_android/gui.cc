#include "gui.h"

#include "log.h"

#include <memory>

using android_ros::GUI;

GUI::GUI() {}

GUI::~GUI() { Stop(); }

void GUI::DrawingLoop(ANativeWindow* window,
                      std::promise<void> promise_first_frame) {
  LOGI("Entered DrawingLoop()");
  // Display initialization MUST happen in drawing thread
  InitializeDisplay(window);
  DrawFrame();
  promise_first_frame.set_value();

  while (not exit_loop_.load()) {
    CheckInput();
    DrawFrame();
  }
}

void GUI::CheckInput() {
  std::unique_lock<std::mutex> lock(iqueue_mtx_, std::defer_lock);
  // Try to get input events, but don't worry about if it doesn't happen
  if (!lock.try_lock()) {
    return;
  }
  if (iqueue_ == nullptr) {
    return;
  }
  AInputEvent* event = nullptr;
  while (AInputQueue_getEvent(iqueue_, &event) >= 0) {
    LOGI("New input event: type=%d\n", AInputEvent_getType(event));
    if (AInputQueue_preDispatchEvent(iqueue_, event)) {
      continue;
    }
    int32_t handled = 0;
    // TODO pass the event to Dear Imgui
    AInputQueue_finishEvent(iqueue_, event, handled);
  }
}

void GUI::SetInputQueue(AInputQueue* queue) {
  std::lock_guard<std::mutex> guard(iqueue_mtx_);
  iqueue_ = queue;
}

void GUI::RemoveInputQueue() {
  std::lock_guard<std::mutex> guard(iqueue_mtx_);
  iqueue_ = nullptr;
}

void GUI::Start(ANativeWindow* window) {
  exit_loop_.store(false);
  std::promise<void> promise_first_frame;
  std::future<void> first_frame_drawn = promise_first_frame.get_future();
  draw_thread_ = std::thread(&GUI::DrawingLoop, this, window,
                             std::move(promise_first_frame));
  first_frame_drawn.wait();
}

void GUI::Stop() {
  if (draw_thread_.joinable()) {
    // Join drawing thread
    exit_loop_.store(true);
    draw_thread_.join();
  }
  TerminateDisplay();
}

bool GUI::InitializeDisplay(ANativeWindow* window) {
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
  surface = eglCreateWindowSurface(display, config, window, nullptr);
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

  LOGI("Display width %d height %d", width_, height_);

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

  if (EGL_TRUE != eglSwapBuffers(display_, surface_)) {
    EGLint egl_error = eglGetError();
    LOGW("Failed to swap EGL buffers %d", egl_error);
  }
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
