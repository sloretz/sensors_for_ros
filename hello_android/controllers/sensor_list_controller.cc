#include "imgui.h"

#include "controllers/sensor_list_controller.h"

namespace android_ros {
SensorListController::SensorListController(android_ros::Sensors& sensors)
  : sensors_(sensors)
{
}

void SensorListController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Sensor List", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);

  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }

  ImGui::Separator();

  // const char* items[] = { "AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "FFFF", "GGGG", "HHHH", "IIII", "JJJJ", "KKKK", "LLLLLLL", "MMMM", "OOOOOOO" };

  // Which sensor is selected
  static int selected_handle = -1;

  if (ImGui::BeginListBox("##sensor_list_box", ImVec2(-FLT_MIN, -FLT_MIN))) {
    const auto & sensors = sensors_.GetSensors();
    for (const auto & sensor : sensors) {
      const SensorDescriptor desc = sensor->Descriptor();
      const bool is_selected = (desc.handle == selected_handle);

      ImGui::PushID(desc.vendor);
      ImGui::PushID(desc.name);

      if (ImGui::Selectable("##selectable", is_selected)) {
          selected_handle = desc.handle;
          auto event = event::GuiNavigateToSensor();
          event.handle = selected_handle;
          Emit(event);
      }

      ImGui::SameLine();
      ImGui::Text("%s: %s", desc.PrettyType(), desc.name);

      ImGui::PopID();
      ImGui::PopID();

      // Set the initial focus when opening the combo (scrolling + keyboard navigation focus)
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndListBox();
  }

  ImGui::End();
}
}  // namespace android_ros
