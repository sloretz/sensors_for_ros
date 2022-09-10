#include "imgui.h"

#include "controllers/illuminance_sensor_controller.h"

namespace android_ros {
IlluminanceSensorController::IlluminanceSensorController(
  IlluminanceSensor* sensor,
  RosInterface& ros)
  : sensor_(sensor), publisher_(ros), Controller(std::string(sensor->Descriptor().name) + sensor->Descriptor().vendor)
{
  sensor->SetListener(
    std::bind(&IlluminanceSensorController::OnIlluminanceChanged, this, std::placeholders::_1));

  // TODO allow publisher topic to be set from GUI
  publisher_.SetTopic("illuminance");
  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void
IlluminanceSensorController::OnIlluminanceChanged(
    const sensor_msgs::msg::Illuminance& msg)
{
  last_msg_ = msg;
  publisher_.Publish(msg);
}

void IlluminanceSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Illuminace Senosr", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Illuminance Sensor");
  ImGui::Separator();
  ImGui::Text("Name: %s", sensor_->Descriptor().name);
  ImGui::Text("Vendor: %s", sensor_->Descriptor().vendor);
  ImGui::Separator();
  ImGui::Text("Last measurement: %.2f lx", last_msg_.illuminance);
  ImGui::End();
}

std::string IlluminanceSensorController::PrettyName() const
{
  return "Light Sensor";
}
}  // namespace android_ros
