#include "imgui.h"

#include "controllers/barometer_sensor_controller.h"

namespace android_ros {
BarometerSensorController::BarometerSensorController(
  BarometerSensor* sensor,
  RosInterface& ros)
  : sensor_(sensor), publisher_(ros)
{
  sensor->SetListener(
    std::bind(&BarometerSensorController::OnSensorReading, this, std::placeholders::_1));

  // TODO allow publisher topic to be set from GUI
  publisher_.SetTopic("barometer");
  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void
BarometerSensorController::OnSensorReading(
    const sensor_msgs::msg::FluidPressure& msg)
{
  last_msg_ = msg;
  // LOGI("Publishing ROS message for Barometer");
  publisher_.Publish(msg);
}

void BarometerSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Barometer", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Barometer");
  ImGui::Separator();
  ImGui::Text("Name: %s", sensor_->Descriptor().name);
  ImGui::Text("Vendor: %s", sensor_->Descriptor().vendor);
  ImGui::Separator();
  ImGui::Text("Last measurement: %.2f Pa", last_msg_.fluid_pressure);
  ImGui::End();
}
}  // namespace android_ros
