//
// Created by timovdk on 2/11/20.
//

#include "stp/new_skills/SetDribbler.h"

namespace rtt::ai::stp {

Status SetDribbler::onInitialize() noexcept {
    return Status::Success;
}
//TODO implement this skill again
Status SetDribbler::onUpdate(const rtt::ai::stp::SkillInfo &info) noexcept {
/*

  if (dribblerSpeed < 0 || dribblerSpeed > 255) {
      return Status::Failure;
  }

  command.set_dribbler(dribblerSpeed);

  publishRobotCommand();
*/

  return Status::Running;
}

Status SetDribbler::onTerminate() noexcept {}

}  // namespace rtt::ai
