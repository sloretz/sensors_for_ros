#pragma once

#include <vector>

#include "controller.h"
#include "events.h"
#include "log.h"

namespace sensors_for_ros {

// There's only supposed to be one of these in the process
constexpr const char *kListControllerId = "list_controller";

class ListController : public Controller,
                       public event::Emitter<event::GuiNavigateBack>,
                       public event::Emitter<event::GuiNavigateTo> {
 public:
  using event::Emitter<event::GuiNavigateBack>::SetListener;
  using event::Emitter<event::GuiNavigateTo>::SetListener;
  using event::Emitter<event::GuiNavigateBack>::Emit;
  using event::Emitter<event::GuiNavigateTo>::Emit;

  ListController();

  void AddController(const Controller *controller);

  virtual ~ListController() = default;

  void DrawFrame() override;

  std::string PrettyName() const override { return "List Controller"; }

 private:
  std::vector<const Controller *> controllers_;
};
}  // namespace sensors_for_ros
