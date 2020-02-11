//
// Created by timovdk on 2/11/20.
//

#include "new_skills/SetDribbler.h"

namespace rtt::ai {

SetDribbler::SetDribbler(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void SetDribbler::onInitialize() { Skill::onInitialize(); }

SetDribbler::Status SetDribbler::onUpdate() {
    command.set_dribbler(properties->getInt("dribblerSpeed"));

    publishRobotCommand();

    return Status::Running;
}

void SetDribbler::onTerminate(Skill::Status) {}

}  // namespace rtt::ai
