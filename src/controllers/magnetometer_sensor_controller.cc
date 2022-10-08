#include "imgui.h"

#include "controllers/magnetometer_sensor_controller.h"
#include "display_topic.h"

namespace android_ros {
MagnetometerSensorController::MagnetometerSensorController(
  MagnetometerSensor* sensor,
  RosInterface& ros)
  : sensor_(sensor), publisher_(ros), Controller(std::string(sensor->Descriptor().name) + sensor->Descriptor().vendor)
{
  sensor->SetListener(
    std::bind(&MagnetometerSensorController::OnSensorReading, this, std::placeholders::_1));

  // TODO allow publisher topic to be set from GUI
  publisher_.SetTopic("magnetometer");
  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void
MagnetometerSensorController::OnSensorReading(
    const sensor_msgs::msg::MagneticField& msg)
{
  last_msg_ = msg;
  publisher_.Publish(msg);
}

void MagnetometerSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Magnetometer", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Magnetometer");
  ImGui::Separator();
  ImGui::Text("Name: %s", sensor_->Descriptor().name);
  ImGui::Text("Vendor: %s", sensor_->Descriptor().vendor);
  ImGui::Separator();
  DisplayTopic("", publisher_);
  ImGui::Separator();
  ImGui::Text(
    "Last measurement: %.2f, %.2f, %.2f microtesla",
    last_msg_.magnetic_field.x * kMicroTeslaPerTesla,
    last_msg_.magnetic_field.y * kMicroTeslaPerTesla,
    last_msg_.magnetic_field.z * kMicroTeslaPerTesla);
  ImGui::End();
}

std::string MagnetometerSensorController::PrettyName() const
{
  return "Magnetometer Sensor";
}
}  // namespace android_ros
