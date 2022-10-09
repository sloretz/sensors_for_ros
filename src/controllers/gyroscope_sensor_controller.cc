#include "controllers/gyroscope_sensor_controller.h"

#include "display_topic.h"
#include "imgui.h"

namespace android_ros {
GyroscopeSensorController::GyroscopeSensorController(GyroscopeSensor* sensor,
                                                     RosInterface& ros)
    : sensor_(sensor),
      publisher_(ros),
      Controller(std::string(sensor->Descriptor().name) +
                 sensor->Descriptor().vendor) {
  sensor->SetListener(std::bind(&GyroscopeSensorController::OnGyroReading, this,
                                std::placeholders::_1));

  // TODO allow publisher topic to be set from GUI
  publisher_.SetTopic("gyroscope");
  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void GyroscopeSensorController::OnGyroReading(
    const geometry_msgs::msg::TwistStamped& msg) {
  last_msg_ = msg;
  publisher_.Publish(msg);
}

void GyroscopeSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Gyroscope", &show_dialog,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Gyroscope");
  ImGui::Separator();
  ImGui::Text("Name: %s", sensor_->Descriptor().name);
  ImGui::Text("Vendor: %s", sensor_->Descriptor().vendor);
  ImGui::Separator();
  DisplayTopic("", publisher_);
  ImGui::Separator();
  ImGui::Text("Last measurement: %.2f, %.2f, %.2f rad/s",
              last_msg_.twist.angular.x, last_msg_.twist.angular.y,
              last_msg_.twist.angular.z);
  ImGui::End();
}

std::string GyroscopeSensorController::PrettyName() const {
  return "Gyroscope Sensor";
}
}  // namespace android_ros
