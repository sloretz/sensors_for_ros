#include "controllers/accelerometer_sensor_controller.h"

#include "display_topic.h"
#include "imgui.h"
#include "pub_sub.h"

namespace sensors_for_ros {
PubSub::PubSub(RosInterface& ros)
    : publisher_(ros), sub_(ros), ros_(ros),
      Controller("Simple PubSub") {
  sub_.SetListener(std::bind(&PubSub::OnMsgReceived,
                                this, std::placeholders::_1));
  publisher_.SetTopic("from_android");
  publisher_.Enable();
}

void PubSub::OnMsgReceived(
  const sensors_for_ros::event::RosStringMessageReceived& event) {
  last_msg_ = event.msg;
  publisher_.Publish(last_msg_);
  now_ = ros_.get_node()->now();
  if (!last_time_.seconds()) { 
      last_time_ = now_;
  }
  else 
  {
    auto duration = now_ - last_time_;
    last_time_ = now_;
    topic_hz_ = 1/duration.seconds();
  }
}

void PubSub::DrawFrame() {
  bool show_dialog = true;
  ImGui::Begin("Simple PubSub", &show_dialog,
               ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize |
                   ImGuiWindowFlags_NoTitleBar);
  if (ImGui::Button("< Back")) {
    LOGI("Asked to go back");
    Emit(event::GuiNavigateBack{});
  }
  ImGui::Text("Simple PubSub");
  ImGui::Separator();
  
  ImGui::Text("Node name: %s", ros_.get_node()->get_name());
  ImGui::Separator();
  ImGui::Text("Topic name: %s", sub_.get_topic().c_str());
  ImGui::Separator();
  ImGui::Text("Topic hz: %.3f", topic_hz_);
  ImGui::Separator();
  ImGui::TextWrapped("Last msg: %s", last_msg_.data.c_str());
  ImGui::Separator();
  ImGui::End();
}

std::string PubSub::PrettyName() const {
  return "Simple PubSub";
}
}  // namespace sensors_for_ros
