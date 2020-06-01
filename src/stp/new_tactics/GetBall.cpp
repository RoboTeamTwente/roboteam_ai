//
// Created by ratoone on 10-03-20.
//

#include "stp/new_tactics/GetBall.h"
#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {
GetBall::GetBall() {
    skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

void GetBall::onInitialize() noexcept {}

void GetBall::onUpdate(Status const &status) noexcept {}

void GetBall::onTerminate() noexcept {}

StpInfo GetBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillInfo = info;
    Vector2 robotPosition = info.getRobot().value()->getPos();
    Vector2 ballPosition = info.getBall().value()->getPos();

    // the robot will go to the position of the ball
    double ballDistance = (ballPosition - robotPosition).length();
    Vector2 newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance) - 0.4*control_constants::CENTER_TO_FRONT;

    if (ballDistance < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillInfo.setAngle((ballPosition - robotPosition).angle());
        skillInfo.setDribblerSpeed(100);
    }

    skillInfo.setPositionToMoveTo(newRobotPosition);

    return skillInfo;
}

bool GetBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool GetBall::shouldTacticReset(const StpInfo &info) noexcept { return !info.getRobot()->hasBall(); }

bool GetBall::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *GetBall::getName() {
    return "Get Ball";
}

}  // namespace rtt::ai::stp::tactic
