#include "controllers/list_controller.h"

#include "imgui.h"

namespace sensors_for_ros {
ListController::ListController() : Controller(kListControllerId) {}

void ListController::AddController(const Controller* controller) {
  if (controllers_.end() ==
      std::find(controllers_.begin(), controllers_.end(), controller)) {
    controllers_.push_back(controller);
  }
}

void ListController::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Sensor List", &show_dialog,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoTitleBar);

  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }

  ImGui::Separator();

  // Which controller is selected
  static const Controller* selected_controller = nullptr;

  if (ImGui::BeginListBox("##list_box", ImVec2(-FLT_MIN, -FLT_MIN))) {
    for (const Controller* controller : controllers_) {
      const bool is_selected = (controller == selected_controller);

      // TODO Unique ID
      ImGui::PushID(controller->UniqueId());

      if (ImGui::Selectable("##selectable", is_selected)) {
        selected_controller = controller;
        auto event = event::GuiNavigateTo();
        event.unique_id = selected_controller->UniqueId();
        Emit(event);
      }

      ImGui::SameLine();
      std::string name = controller->PrettyName();
      ImGui::Text("%s", name.c_str());

      ImGui::PopID();

      // Set the initial focus when opening the combo (scrolling + keyboard
      // navigation focus)
      if (is_selected) {
        ImGui::SetItemDefaultFocus();
      }
    }
    ImGui::EndListBox();
  }

  ImGui::End();
}
}  // namespace sensors_for_ros
