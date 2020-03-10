//
// Created by ratoone on 10-03-20.
//

#include "stp/new_tactics/GetBall.h"
#include <roboteam_utils/Print.h>
#include "stp/new_skills/GoToPos.h"

namespace rtt::ai::stp {
void GetBall::onInitialize() noexcept {
    skills = collections::state_machine<Skill, Status, SkillInfo>{GoToPos()};

    skills.initialize();
}

void GetBall::onUpdate(Status const &status) noexcept {}

void GetBall::onTerminate() noexcept {}

SkillInfo GetBall::calculateInfoForSkill(TacticInfo const &info) noexcept {
    if (!info.ball) {
        RTT_WARNING("No Ball present in TacticInfo");
        return {};
    }

    SkillInfo skillInfo = SkillInfo();

//    Vector2 newRobotPosition = skillInfo.setRobot();

//    skillInfo.getTacticInfo().setPosition();
    return {};
}
}  // namespace rtt::ai::stp
