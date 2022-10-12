#include <android/native_activity.h>

#include <memory>
#include <vector>

#include "camera_descriptor.h"
#include "camera_manager.h"
#include "controller.h"
#include "controllers/accelerometer_sensor_controller.h"
#include "controllers/barometer_sensor_controller.h"
#include "controllers/camera_controller.h"
#include "controllers/gyroscope_sensor_controller.h"
#include "controllers/illuminance_sensor_controller.h"
#include "controllers/list_controller.h"
#include "controllers/magnetometer_sensor_controller.h"
#include "controllers/ros_domain_id_controller.h"
#include "events.h"
#include "gui.h"
#include "jvm.h"
#include "log.h"
#include "ros_interface.h"
#include "sensors.h"

// Controller class links all the parts
class AndroidApp {
 public:
  AndroidApp(ANativeActivity* activity)
      : activity_(activity), sensors_(activity) {
    ros_domain_id_controller_.SetListener(std::bind(
        &AndroidApp::OnRosDomainIdChanged, this, std::placeholders::_1));
    PushController(&ros_domain_id_controller_);

    list_controller_.SetListener(
        std::bind(&AndroidApp::OnNavigateBack, this, std::placeholders::_1));
    list_controller_.SetListener(
        std::bind(&AndroidApp::OnNavigateTo, this, std::placeholders::_1));

    LOGI("Initalizing Sensors");
    sensors_.Initialize();

    // Create sensor-specific controllers
    for (auto& sensor : sensors_.GetSensors()) {
      if (ASENSOR_TYPE_LIGHT == sensor->Descriptor().type) {
        auto controller =
            std::make_unique<sensors_for_ros::IlluminanceSensorController>(
                static_cast<sensors_for_ros::IlluminanceSensor*>(sensor.get()),
                ros_);
        // Listen to go-back-to-the-last-window GUI events from this controller
        controller->SetListener(std::bind(&AndroidApp::OnNavigateBack, this,
                                          std::placeholders::_1));
        controllers_.emplace_back(std::move(controller));
        LOGI("Sensor controller with handle %d added",
             sensor->Descriptor().handle);
      } else if (ASENSOR_TYPE_GYROSCOPE == sensor->Descriptor().type) {
        auto controller =
            std::make_unique<sensors_for_ros::GyroscopeSensorController>(
                static_cast<sensors_for_ros::GyroscopeSensor*>(sensor.get()),
                ros_);
        // Listen to go-back-to-the-last-window GUI events from this controller
        controller->SetListener(std::bind(&AndroidApp::OnNavigateBack, this,
                                          std::placeholders::_1));
        controllers_.emplace_back(std::move(controller));
        LOGI("Sensor controller with handle %d added",
             sensor->Descriptor().handle);
      } else if (ASENSOR_TYPE_ACCELEROMETER == sensor->Descriptor().type) {
        auto controller =
            std::make_unique<sensors_for_ros::AccelerometerSensorController>(
                static_cast<sensors_for_ros::AccelerometerSensor*>(
                    sensor.get()),
                ros_);
        // Listen to go-back-to-the-last-window GUI events from this controller
        controller->SetListener(std::bind(&AndroidApp::OnNavigateBack, this,
                                          std::placeholders::_1));
        controllers_.emplace_back(std::move(controller));
        LOGI("Sensor controller with handle %d added",
             sensor->Descriptor().handle);
      } else if (ASENSOR_TYPE_PRESSURE == sensor->Descriptor().type) {
        auto controller =
            std::make_unique<sensors_for_ros::BarometerSensorController>(
                static_cast<sensors_for_ros::BarometerSensor*>(sensor.get()),
                ros_);
        // Listen to go-back-to-the-last-window GUI events from this controller
        controller->SetListener(std::bind(&AndroidApp::OnNavigateBack, this,
                                          std::placeholders::_1));
        controllers_.emplace_back(std::move(controller));
        LOGI("Sensor controller with handle %d added",
             sensor->Descriptor().handle);
      } else if (ASENSOR_TYPE_MAGNETIC_FIELD == sensor->Descriptor().type) {
        auto controller =
            std::make_unique<sensors_for_ros::MagnetometerSensorController>(
                static_cast<sensors_for_ros::MagnetometerSensor*>(sensor.get()),
                ros_);
        // Listen to go-back-to-the-last-window GUI events from this controller
        controller->SetListener(std::bind(&AndroidApp::OnNavigateBack, this,
                                          std::placeholders::_1));
        controllers_.emplace_back(std::move(controller));
        LOGI("Sensor controller with handle %d added",
             sensor->Descriptor().handle);
      }
    }

    for (const auto& controller : controllers_) {
      list_controller_.AddController(controller.get());
    }

    // Create camera specific controllers
    if (camera_manager_.HasCameras()) {
      if (!sensors_for_ros::HasPermission(activity_, "CAMERA")) {
        LOGI("Requesting Camera Permission");
        sensors_for_ros::RequestPermission(activity_, "CAMERA");
      } else {
        StartCameras();
      }
    }
  }

  ~AndroidApp() = default;

  // Camera permission has to be requested at runtime, and we're not sure when
  // we'll get it.
  // Try to start the cameras.
  void StartCameras() {
    if (started_cameras_) {
      return;
    }
    if (camera_manager_.HasCameras() &&
        sensors_for_ros::HasPermission(activity_, "CAMERA")) {
      started_cameras_ = true;
      // Create camera specific controllers
      LOGI("Starting cameras");
      std::vector<sensors_for_ros::CameraDescriptor> cameras =
          camera_manager_.GetCameras();
      for (auto cam_desc : cameras) {
        LOGI("Camera: %s", cam_desc.GetName().c_str());
        std::unique_ptr<sensors_for_ros::CameraController> camera_controller(
            new sensors_for_ros::CameraController(&camera_manager_, cam_desc,
                                                  ros_));
        camera_controller->SetListener(std::bind(&AndroidApp::OnNavigateBack,
                                                 this, std::placeholders::_1));
        controllers_.emplace_back(std::move(camera_controller));
        list_controller_.AddController(controllers_.back().get());
      }
    }
  }

  ANativeActivity* activity_;
  sensors_for_ros::RosInterface ros_;
  sensors_for_ros::Sensors sensors_;
  sensors_for_ros::GUI gui_;

  // Special controller: ROS_DOMAIN_ID picker shown at startup
  sensors_for_ros::RosDomainIdController ros_domain_id_controller_;
  // Special controller: Show list of sensors and cameras
  sensors_for_ros::ListController list_controller_;
  // Controllers that can be nativated to by unique id
  std::vector<std::unique_ptr<sensors_for_ros::Controller>> controllers_;

  // Stack of controllers for navigation windows
  std::vector<sensors_for_ros::Controller*> controller_stack_;

  // Manager for working for cameras
  sensors_for_ros::CameraManager camera_manager_;
  bool started_cameras_ = false;

 private:
  void PushController(sensors_for_ros::Controller* controller) {
    if (controller) {
      controller_stack_.push_back(controller);
      gui_.SetController(controller);
    }
  }

  void PopController() {
    // Don't allow popping past the first controller
    if (controller_stack_.size() > 1) {
      controller_stack_.pop_back();
      gui_.SetController(controller_stack_.back());
    }
  }

  void OnNavigateBack(const sensors_for_ros::event::GuiNavigateBack&) {
    LOGI("Poping controller!");
    PopController();
  }

  void OnNavigateTo(const sensors_for_ros::event::GuiNavigateTo& event) {
    auto cit = std::find_if(controllers_.begin(), controllers_.end(),
                            [&event](const auto& other) {
                              return event.unique_id == other->UniqueId();
                            });
    if (cit != controllers_.end()) {
      PushController(cit->get());
    }
  }

  void OnRosDomainIdChanged(
      const sensors_for_ros::event::RosDomainIdChanged& event) {
    StartRos(event.id);
    PushController(&list_controller_);
  }

  void StartRos(int32_t ros_domain_id) {
    if (ros_.Initialized()) {
      LOGI("Shutting down ROS");
      ros_.Shutdown();
    }
    LOGI("Initalizing ROS");
    ros_.Initialize(ros_domain_id);
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
  GetApp(activity)->sensors_.Shutdown();
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
static void onPause(ANativeActivity* activity) {
  LOGI("Pause: %p\n", activity);
}

/// NativeActivity has resumed.
static void onResume(ANativeActivity* activity) {
  LOGI("Resume: %p\n", activity);
  GetApp(activity)->StartCameras();
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
  activity->instance = new AndroidApp(activity);
}
