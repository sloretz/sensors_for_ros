#include "controllers/accelerometer_sensor_controller.h"

#include "display_topic.h"
#include "imgui.h"

namespace sensors_for_ros {
AccelerometerSensorController::AccelerometerSensorController(
    AccelerometerSensor* sensor, RosInterface& ros)
    : sensor_(sensor),
      publisher_(ros),
      Controller(std::string(sensor->Descriptor().name) +
                 sensor->Descriptor().vendor) {
  sensor->SetListener(std::bind(&AccelerometerSensorController::OnSensorReading,
                                this, std::placeholders::_1));

  // TODO allow publisher topic to be set from GUI
  publisher_.SetTopic("accelerometer");
  // TODO allow publisher to be enabled/disabled from GUI
  publisher_.Enable();

  // TODO allow GUI to change topic and QoS
}

void AccelerometerSensorController::OnSensorReading(
    const geometry_msgs::msg::AccelStamped& msg) {
  last_msg_ = msg;
  publisher_.Publish(msg);
}

void AccelerometerSensorController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Accelerometer", &show_dialog,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Accelerometer");
  ImGui::Separator();
  ImGui::Text("Name: %s", sensor_->Descriptor().name);
  ImGui::Text("Vendor: %s", sensor_->Descriptor().vendor);
  ImGui::Separator();
  DisplayTopic("", publisher_);
  ImGui::Separator();
  ImGui::Text("Last measurement: %.2f, %.2f, %.2f m/s^2",
              last_msg_.accel.linear.x, last_msg_.accel.linear.y,
              last_msg_.accel.linear.z);
  ImGui::End();
}

std::string AccelerometerSensorController::PrettyName() const {
  return "Accelerometer Sensor";
}
}  // namespace sensors_for_ros
