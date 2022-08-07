#include "imgui.h"

#include "illuminance_sensor_controller.h"

namespace android_ros {
IlluminanceSensorController::IlluminanceSensorController(
  IlluminanceSensor* sensor,
  Publisher<sensor_msgs::msg::Illuminance> publisher)
  : sensor_(sensor), publisher_(std::move(publisher))
{
  sensor->SetListener(
    std::bind(&IlluminanceSensorController::OnIlluminanceChanged, this, std::placeholders::_1));

  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void
IlluminanceSensorController::OnIlluminanceChanged(
    const event::IlluminanceChanged& event)
{
  auto msg = sensor_msgs::msg::Illuminance();
  // TODO(sloretz) time and frame id
  msg.illuminance = event.light;
  msg.variance = 0.0;
  publisher_.Publish(msg);
  LOGI("Publishing ROS message %lf lx", msg.illuminance);
}

void IlluminanceSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Illuminace Senosr", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::End();
}
}  // namespace android_ros
