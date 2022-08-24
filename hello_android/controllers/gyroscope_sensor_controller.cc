#include "imgui.h"

#include "controllers/gyroscope_sensor_controller.h"

namespace android_ros {
GyroscopeSensorController::GyroscopeSensorController(
  GyroscopeSensor* sensor,
  Publisher<geometry_msgs::msg::TwistStamped> publisher)
  : sensor_(sensor), publisher_(std::move(publisher))
{
  sensor->SetListener(
    std::bind(&GyroscopeSensorController::OnGyroReading, this, std::placeholders::_1));

  // TODO allow publisher topic to be set from GUI
  publisher_.SetTopic("gyroscope");
  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void
GyroscopeSensorController::OnGyroReading(
    const geometry_msgs::msg::TwistStamped& msg)
{
  last_msg_ = msg;
  LOGI("Publishing ROS message for gyro");
  publisher_.Publish(msg);
}

void GyroscopeSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Gyroscope", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Gyroscope");
  ImGui::Separator();
  ImGui::Text("Name: %s", sensor_->Descriptor().name);
  ImGui::Text("Vendor: %s", sensor_->Descriptor().vendor);
  ImGui::Separator();
  ImGui::Text(
    "Last measurement: %.2f, %.2f, %.2f rad/s",
    last_msg_.twist.angular.x,
    last_msg_.twist.angular.y,
    last_msg_.twist.angular.z);
  ImGui::End();
}
}  // namespace android_ros

