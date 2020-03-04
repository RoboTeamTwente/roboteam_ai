//
// Created by timovdk on 2/11/20.
//

#include "stp/new_skills/SetDribbler.h"

namespace rtt::ai {

SetDribbler::SetDribbler(std::string name, bt::Blackboard::Ptr blackboard)
    : Skill(std::move(name), std::move(blackboard)) {}

void SetDribbler::onInitialize() { Skill::onInitialize(); }

SetDribbler::Status SetDribbler::onUpdate() {
  int dribblerSpeed = properties->getInt("dribblerSpeed");

  if (dribblerSpeed < 0 || dribblerSpeed > 255) {
      return Status::Failure;
  }

  command.set_dribbler(dribblerSpeed);

  publishRobotCommand();

  return Status::Running;
}

void SetDribbler::onTerminate(Skill::Status) {}

}  // namespace rtt::ai
