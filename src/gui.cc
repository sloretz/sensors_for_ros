#include "gui.h"

#include <android/native_window.h>

#include <memory>

#include "imgui.h"
#include "imgui_impl_android.h"
#include "imgui_impl_opengl3.h"
#include "log.h"

using android_ros::GUI;

GUI::GUI() {}

GUI::~GUI() { Stop(); }

void GUI::SetController(Controller* controller) {
  active_controller = controller;
}

void GUI::DrawingLoop(ANativeWindow* window,
                      std::promise<void> promise_first_frame) {
  LOGI("Entered DrawingLoop()");
  // Display initialization MUST happen in drawing thread
  InitializeDisplay(window);
  InitializeDearImGui(window);
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
    handled = ImGui_ImplAndroid_HandleInputEvent(event);
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

void GUI::Start(ANativeActivity* activity, ANativeWindow* window) {
  activity_ = activity;
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
  activity_ = nullptr;
}

bool GUI::InitializeDisplay(ANativeWindow* window) {
  // Copied from
  // https://developer.android.com/ndk/samples/sample_na
  // and
  // https://github.com/ocornut/imgui/blob/e346059eef140c5a8611581f3e6c8b8816d6998e/examples/example_android_opengl3/main.cpp#L51-L65

  const EGLint egl_attributes[] = {EGL_BLUE_SIZE,    8,
                                   EGL_GREEN_SIZE,   8,
                                   EGL_RED_SIZE,     8,
                                   EGL_DEPTH_SIZE,   24,
                                   EGL_SURFACE_TYPE, EGL_WINDOW_BIT,
                                   EGL_NONE};
  EGLint w, h, format;
  EGLint numConfigs;
  EGLConfig config = nullptr;
  EGLSurface surface;
  EGLContext context;

  EGLDisplay display = eglGetDisplay(EGL_DEFAULT_DISPLAY);

  eglInitialize(display, nullptr, nullptr);

  // Get the first matching config
  EGLConfig egl_config;
  eglChooseConfig(display, egl_attributes, &egl_config, 1, &numConfigs);
  EGLint egl_format;
  eglGetConfigAttrib(display, egl_config, EGL_NATIVE_VISUAL_ID, &egl_format);
  ANativeWindow_setBuffersGeometry(window, 0, 0, egl_format);

  const EGLint egl_context_attributes[] = {EGL_CONTEXT_CLIENT_VERSION, 3,
                                           EGL_NONE};
  context = eglCreateContext(display, egl_config, EGL_NO_CONTEXT,
                             egl_context_attributes);

  if (context == EGL_NO_CONTEXT) {
    LOGW("eglCreateContext() returned EGL_NO_CONTEXT");
  }

  surface = eglCreateWindowSurface(display, egl_config, window, NULL);
  eglMakeCurrent(display, surface, surface, context);

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

  return true;
}

void GUI::InitializeDearImGui(ANativeWindow* window) {
  // Setup Dear ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO& io = ImGui::GetIO();

  // Disable loading/saving of .ini file from disk.
  io.IniFilename = NULL;

  // Setup Dear ImGui style
  ImGui::StyleColorsDark();

  // Setup Platform/Renderer backends
  ImGui_ImplAndroid_Init(window);
  ImGui_ImplOpenGL3_Init("#version 300 es");

  // TODO(sloretz) DPI aware scaling
  ImFontConfig font_cfg;
  font_cfg.SizePixels = 50.0f;
  io.Fonts->AddFontDefault(&font_cfg);
  ImGui::GetStyle().ScaleAllSizes(4.0f);
}

void GUI::TerminateDearImGui() {
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplAndroid_Shutdown();
  ImGui::DestroyContext();
}

void GUI::DrawFrame() {
  ImGuiIO& io = ImGui::GetIO();
  if (display_ == EGL_NO_DISPLAY) {
    return;
  }

  // Start the Dear ImGui frame
  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplAndroid_NewFrame();
  ImGui::NewFrame();

  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
  if (io.DisplaySize.x <= 0 || io.DisplaySize.y <= 0) {
    LOGI("huyh, don't know size yet?");
  } else {
    // Each controller runs full screen
    ImGui::SetNextWindowSize(io.DisplaySize);
    // Add some padding
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding,
                        ImVec2(io.DisplaySize.x / 10, io.DisplaySize.x / 10));

    if (active_controller) {
      active_controller->DrawFrame();
    }

    ImGui::PopStyleVar();
  }

  // Rendering
  ImGui::Render();
  glViewport(0, 0, (int)io.DisplaySize.x, (int)io.DisplaySize.y);
  glClearColor(0.8f, 0.8f, 0.8f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
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
