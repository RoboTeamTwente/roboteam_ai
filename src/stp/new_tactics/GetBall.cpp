//
// Created by ratoone on 10-03-20.
//

#include "stp/new_tactics/GetBall.h"
#include <roboteam_utils/Print.h>
#include "stp/new_skills/Rotate.h"
#include "stp/new_skills/GoToPos.h"

namespace rtt::ai::stp {
void GetBall::onInitialize() noexcept {
    skills = collections::state_machine<Skill, Status, SkillInfo>{GoToPos(), Rotate()};

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
    Vector2 robotPosition;
    Vector2 ballPosition = info.ball.value()->getPos();
    // the robot will go to the position of the ball
    double ballDistance = (ballPosition - robotPosition).length();
    Vector2 newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance - Constants::ROBOT_RADIUS());
    if (ballDistance < 3 * Constants::ROBOT_RADIUS()){
        skillInfo.setAngle((ballPosition - robotPosition).angle());
        // TODO: what is the dribbler speed? why is the dribbler speed?
        skillInfo.setDribblerSpeed(100);
    }

//    skillInfo.getTacticInfo().setPosition();
    return skillInfo;
}
}  // namespace rtt::ai::stp
