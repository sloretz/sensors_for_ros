#include "imgui.h"

#include <sstream>

#include "camera_controller.h"

using android_ros::CameraController;
using android_ros::CameraDevice;

CameraController::CameraController(
      std::unique_ptr<CameraDevice> device,
      RosInterface& ros
) : device_(std::move(device)), Controller(device->GetDescriptor().GetName()), info_pub_(ros), image_pub_(ros)
{
  device_->SetListener(
    std::bind(&CameraController::OnImage, this, std::placeholders::_1));

  auto desc = device_->GetDescriptor();
  std::stringstream base_topic;
  // TODO(sloretz) what if id has invalid characters?
  base_topic << "camera/id_" << desc.id << "/";

  std::string info_topic = base_topic.str() + "camera_info";
  std::string image_topic = base_topic.str() + "image_color";

  // TODO allow publisher topic to be set from GUI
  info_pub_.SetTopic(info_topic.c_str());
  // TODO allow publisher to be enabled/disabled from GUI
  info_pub_.Enable();

  // TODO allow publisher topic to be set from GUI
  image_pub_.SetTopic(image_topic.c_str());
  image_pub_.SetQos(rclcpp::QoS(1).best_effort());
  // TODO allow publisher to be enabled/disabled from GUI
  image_pub_.Enable();
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

void
CameraController::OnImage(const std::pair<CameraInfo::UniquePtr, Image::UniquePtr> & info_image)
{
  LOGI("Controller has image?");
  // TODO move images if I ever need intraprocesses stuff - requires changes to Emitter class
  info_pub_.Publish(*info_image.first.get());
  image_pub_.Publish(*info_image.second.get());
}
