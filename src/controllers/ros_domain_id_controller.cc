#include "controllers/ros_domain_id_controller.h"

#include "imgui.h"

namespace android_ros {
void RosDomainIdController::DrawFrame() {
  static int32_t picked_ros_domain_id = -1;

  auto increase_id = [](int num) {
    if (picked_ros_domain_id < 0) {
      picked_ros_domain_id = num;
    } else {
      picked_ros_domain_id = picked_ros_domain_id * 10 + num;
    }
  };

  bool show_ros_domain_id_picker = true;
  ImGui::Begin("ROS_DOMAIN_ID", &show_ros_domain_id_picker,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoTitleBar);

  // Disable buttons so that only domain ids between 0 and 232 are pickable

  bool disabled = false;
#define DISABLE_IF(expr)       \
  {                            \
    if (!disabled && (expr)) { \
      ImGui::BeginDisabled();  \
      disabled = true;         \
    }                          \
  }                            \
  while (false)

  DISABLE_IF(picked_ros_domain_id >= 30);
  if (ImGui::Button("0")) {
    increase_id(0);
  }
  ImGui::SameLine();
  if (ImGui::Button("1")) {
    increase_id(1);
  }
  ImGui::SameLine();
  if (ImGui::Button("2")) {
    increase_id(2);
  }
  DISABLE_IF(picked_ros_domain_id == 23);
  if (ImGui::Button("3")) {
    increase_id(3);
  }
  ImGui::SameLine();
  if (ImGui::Button("4")) {
    increase_id(4);
  }
  ImGui::SameLine();
  if (ImGui::Button("5")) {
    increase_id(5);
  }
  if (ImGui::Button("6")) {
    increase_id(6);
  }
  ImGui::SameLine();
  if (ImGui::Button("7")) {
    increase_id(7);
  }
  ImGui::SameLine();
  if (ImGui::Button("8")) {
    increase_id(8);
  }
  if (ImGui::Button("9")) {
    increase_id(9);
  }

  if (disabled) {
    ImGui::EndDisabled();
  }

  ImGui::SameLine();
  if (ImGui::Button("Clear")) {
    picked_ros_domain_id = -1;
  }

  if (picked_ros_domain_id < 0) {
    ImGui::Text("---");
  } else {
    ImGui::Text("%d", picked_ros_domain_id);
  }

  if (ImGui::Button("Set ROS_DOMAIN_ID")) {
    if (picked_ros_domain_id < 0) {
      // Default to 0
      picked_ros_domain_id = 0;
    }
    Emit(event::RosDomainIdChanged{picked_ros_domain_id});
  }
  ImGui::End();
}
}  // namespace android_ros
