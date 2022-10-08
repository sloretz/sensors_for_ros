#include "imgui.h"

#include <sstream>

#include "camera_controller.h"
#include "display_topic.h"

using android_ros::CameraController;
using android_ros::CameraDevice;

CameraController::CameraController(
      CameraManager* camera_manager,
      const CameraDescriptor& camera_descriptor,
      RosInterface& ros
) : camera_manager_(camera_manager), camera_descriptor_(camera_descriptor), Controller(camera_descriptor.GetName()), info_pub_(ros), image_pub_(ros)
{
  std::stringstream base_topic;
  // TODO(sloretz) what if id has invalid characters?
  base_topic << "camera/id_" << camera_descriptor_.id << "/";

  std::string info_topic = base_topic.str() + "camera_info";
  std::string image_topic = base_topic.str() + "image_color";

  // TODO allow publisher topic to be set from GUI
  info_pub_.SetTopic(info_topic.c_str());

  // TODO allow publisher topic to be set from GUI
  image_pub_.SetTopic(image_topic.c_str());
  image_pub_.SetQos(rclcpp::QoS(1).best_effort());
}

CameraController::~CameraController()
{
}

void CameraController::EnableCamera()
{
  image_pub_.Enable();
  info_pub_.Enable();
  device_ = camera_manager_->OpenCamera(camera_descriptor_);
  device_->SetListener(
    std::bind(&CameraController::OnImage, this, std::placeholders::_1));
}

void CameraController::DisableCamera()
{
  image_pub_.Disable();
  info_pub_.Disable();
  device_.reset();
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
  ImGui::Text("%s", camera_descriptor_.GetName().c_str());

  ImGui::Separator();

  if (image_pub_.Enabled() && ImGui::Button("Disable")) {
    DisableCamera();
  } else if (!image_pub_.Enabled() && ImGui::Button("Enable")) {
    EnableCamera();
  }

  ImGui::Separator();

  DisplayTopic("Image", image_pub_);

  ImGui::Spacing();

  DisplayTopic("Camera Info", info_pub_);

  if (device_) {
    ImGui::Separator();
    auto [width, height] = device_->Resolution();
    ImGui::Text("Resolution: %dx%d", width, height);
  }

  ImGui::End();
}

std::string CameraController::PrettyName() const
{
  std::string name{camera_descriptor_.GetName()};
  if (!device_) {
    name += " [disabled]";
  }
  return name;
}

void
CameraController::OnImage(const std::pair<CameraInfo::UniquePtr, Image::UniquePtr> & info_image)
{
  LOGI("Controller has image?");
  // TODO move images if I ever need intraprocesses stuff - requires changes to Emitter class
  info_pub_.Publish(*info_image.first.get());
  image_pub_.Publish(*info_image.second.get());
}
