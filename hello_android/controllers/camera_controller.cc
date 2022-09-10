#include "imgui.h"

#include "camera_controller.h"

using android_ros::CameraController;
using android_ros::CameraDevice;

CameraController::CameraController(
      std::unique_ptr<CameraDevice> device,
      RosInterface& ros
) : device_(std::move(device)), Controller(device->GetDescriptor().GetName())
{
}

CameraController::~CameraController()
{
}

// Called by the GUI to draw a frame
void CameraController::DrawFrame()
{
  bool show_dialog = true;
  ImGui::Begin("Camera", &show_dialog,
    ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Camera TODO");
  ImGui::End();
}

std::string CameraController::PrettyName() const
{
  return device_->GetDescriptor().GetName();
}
