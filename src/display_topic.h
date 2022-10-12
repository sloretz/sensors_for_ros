#pragma once

#include "imgui.h"
#include "ros_interface.h"

namespace sensors_for_ros {
template <typename MsgT>
void DisplayTopic(const char* title, const Publisher<MsgT>& pub) {
  std::string title_str{title};
  title_str += " Topic";
  if (ImGui::CollapsingHeader(title_str.c_str(), ImGuiTreeNodeFlags_None)) {
    ImGui::TextWrapped("%s", pub.Topic());
    ImGui::TextWrapped("%s", pub.Type());
  }
}
}  // namespace sensors_for_ros
