#include <memory>

#include <android/native_activity.h>

#include "events.h"
#include "gui.h"
#include "log.h"
#include "ros_interface.h"

// Controller class links all the parts
class AndroidApp : public android_ros::event::Listener {
 public:
  AndroidApp() {
    gui_.SetListener(
        std::bind(&AndroidApp::OnGUIEvent, this, std::placeholders::_1));
  }

  ~AndroidApp() = default;

  android_ros::ROSInterface ros_;
  android_ros::GUI gui_;

 private:
  void OnGUIEvent(const android_ros::event::Event& event) {
    LOGI("Got GUI event");

    std::visit(
        [this](auto&& e) {
          using T = std::decay_t<decltype(e)>;
          if constexpr (std::is_same_v<
                            T, android_ros::event::RosDomainIdChanged>) {
            LOGI("New ROS_DOMAIN_ID %d", e.id);
            if (ros_.Initialized()) {
              ros_.Shutdown();
            }
            ros_.Initialize(e.id);
          } else {
            LOGW("Unknown GUI event");
          }
        },
        event);
  }
};

inline AndroidApp* GetApp(ANativeActivity* activity) {
  return static_cast<AndroidApp*>(activity->instance);
}

/// The current device AConfiguration has changed.
static void onConfigurationChanged(ANativeActivity* activity) {
  LOGI("ConfigurationChanged: %p\n", activity);
}

/// The rectangle in the window in which content should be placed has changed.
static void onContentRectChanged(ANativeActivity* activity, const ARect* rect) {
  LOGI("ContentRectChanged: %p\n", activity);
}

/// NativeActivity is being destroyed.
static void onDestroy(ANativeActivity* activity) {
  LOGI("Destroy: %p\n", activity);
}

/// The input queue for this native activity's window has been created.
static void onInputQueueCreated(ANativeActivity* activity, AInputQueue* queue) {
  LOGI("InputQueueCreated: %p -- %p\n", activity, queue);
  GetApp(activity)->gui_.SetInputQueue(queue);
}

/// The input queue for this native activity's window is being destroyed.
static void onInputQueueDestroyed(ANativeActivity* activity,
                                  AInputQueue* queue) {
  LOGI("InputQueueDestroyed: %p -- %p\n", activity, queue);
  GetApp(activity)->gui_.RemoveInputQueue();
}

/// The system is running low on memory.
static void onLowMemory(ANativeActivity* activity) {
  LOGI("LowMemory: %p\n", activity);
}

/// The drawing window for this native activity has been created.
static void onNativeWindowCreated(ANativeActivity* activity,
                                  ANativeWindow* window) {
  LOGI("NativeWindowCreated: %p -- %p\n", activity, window);
  GetApp(activity)->gui_.Start(activity, window);
}

/// The drawing window for this native activity is going to be destroyed.
static void onNativeWindowDestroyed(ANativeActivity* activity,
                                    ANativeWindow* window) {
  LOGI("NativeWindowDestroyed: %p -- %p\n", activity, window);
  GetApp(activity)->gui_.Stop();
}

/// The drawing window for this native activity needs to be redrawn.
static void onNativeWindowRedrawNeeded(ANativeActivity* activity,
                                       ANativeWindow* window) {
  LOGI("NativeWindowRedrawNeeded: %p -- %p\n", activity, window);
}

/// The drawing window for this native activity has been resized.
static void onNativeWindowResized(ANativeActivity* activity,
                                  ANativeWindow* window) {
  LOGI("NativeWindowResized: %p -- %p\n", activity, window);
}

/// NativeActivity has paused.
static void onPause(ANativeActivity* activity) {}

/// NativeActivity has resumed.
static void onResume(ANativeActivity* activity) {
  LOGI("Resume: %p\n", activity);
}

/// Framework is asking NativeActivity to save its current instance state.
static void* onSaveInstanceState(ANativeActivity* activity, size_t* outLen) {
  LOGI("SaveInstanceState: %p\n", activity);
  return NULL;
}

/// NativeActivity has started.
static void onStart(ANativeActivity* activity) {
  LOGI("Start: %p\n", activity);
}

/// NativeActivity has stopped.
static void onStop(ANativeActivity* activity) { LOGI("Stop: %p\n", activity); }

/// Focus has changed in this NativeActivity's window.
static void onWindowFocusChanged(ANativeActivity* activity, int focused) {
  LOGI("WindowFocusChanged: %p -- %d\n", activity, focused);
}

/// Entry point called by Android to start this app
void ANativeActivity_onCreate(ANativeActivity* activity, void* savedState,
                              size_t savedStateSize) {
  // TODO(sloretz) support saved state - things like ROS domain id and
  // settings
  (void)savedState;
  (void)savedStateSize;

  // https://developer.android.com/ndk/reference/struct/a-native-activity-callbacks
  activity->callbacks->onConfigurationChanged = onConfigurationChanged;
  activity->callbacks->onContentRectChanged = onContentRectChanged;
  activity->callbacks->onDestroy = onDestroy;
  activity->callbacks->onStart = onStart;
  activity->callbacks->onResume = onResume;
  activity->callbacks->onSaveInstanceState = onSaveInstanceState;
  activity->callbacks->onPause = onPause;
  activity->callbacks->onStop = onStop;
  activity->callbacks->onLowMemory = onLowMemory;
  activity->callbacks->onWindowFocusChanged = onWindowFocusChanged;
  activity->callbacks->onNativeWindowCreated = onNativeWindowCreated;
  activity->callbacks->onNativeWindowDestroyed = onNativeWindowDestroyed;
  activity->callbacks->onNativeWindowRedrawNeeded = onNativeWindowRedrawNeeded;
  activity->callbacks->onNativeWindowResized = onNativeWindowResized;
  activity->callbacks->onInputQueueCreated = onInputQueueCreated;
  activity->callbacks->onInputQueueDestroyed = onInputQueueDestroyed;

  // User data
  activity->instance = new AndroidApp();
}