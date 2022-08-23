#include "imgui.h"

#include "accelerometer_sensor_controller.h"

namespace android_ros {
AccelerometerSensorController::AccelerometerSensorController(
  AccelerometerSensor* sensor,
  Publisher<geometry_msgs::msg::AccelStamped> publisher)
  : sensor_(sensor), publisher_(std::move(publisher))
{
  sensor->SetListener(
    std::bind(&AccelerometerSensorController::OnSensorReading, this, std::placeholders::_1));

  // TODO allow publisher topic to be set from GUI
  publisher_.SetTopic("accelerometer");
  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void
AccelerometerSensorController::OnSensorReading(
    const geometry_msgs::msg::AccelStamped& msg)
{
  last_msg_ = msg;
  LOGI("Publishing ROS message for accelerometer");
  publisher_.Publish(msg);
}

void AccelerometerSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Accelerometer", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Accelerometer");
  ImGui::Separator();
  ImGui::Text("Name: %s", sensor_->Descriptor().name);
  ImGui::Text("Vendor: %s", sensor_->Descriptor().vendor);
  ImGui::Separator();
  ImGui::Text(
    "Last measurement: %.2f, %.2f, %.2f m/s^2",
    last_msg_.accel.linear.x,
    last_msg_.accel.linear.y,
    last_msg_.accel.linear.z);
  ImGui::End();
}
}  // namespace android_ros
