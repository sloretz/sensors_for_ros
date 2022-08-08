#include "imgui.h"

#include "sensor_list_controller.h"

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

  const char* items[] = { "AAAA", "BBBB", "CCCC", "DDDD", "EEEE", "FFFF", "GGGG", "HHHH", "IIII", "JJJJ", "KKKK", "LLLLLLL", "MMMM", "OOOOOOO" };

  static int item_current_idx = 0; // Here we store our selection data as an index.

  if (ImGui::BeginListBox("##listbox 2", ImVec2(-FLT_MIN, 5 * ImGui::GetTextLineHeightWithSpacing())))
  {
      for (int n = 0; n < IM_ARRAYSIZE(items); n++)
      {
          const bool is_selected = (item_current_idx == n);
          if (ImGui::Selectable(items[n], is_selected)) {
              item_current_idx = n;
              // TODO Handle from descriptor
              Emit(event::GuiNavigateToSensor{});
          }

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
