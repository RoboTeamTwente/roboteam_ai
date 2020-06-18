//
// Created by ratoone on 10-03-20.
//

#include "stp/new_tactics/GetBall.h"

#include <utilities/GameStateManager.hpp>

#include "control/ControlUtils.h"
#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"
#include "world/FieldComputations.h"

namespace rtt::ai::stp::tactic {
GetBall::GetBall() {
    skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

void GetBall::onInitialize() noexcept {}

void GetBall::onUpdate(Status const &status) noexcept {}

void GetBall::onTerminate() noexcept {}

std::optional<StpInfo> GetBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if(!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    Vector2 robotPosition = info.getRobot().value()->getPos();
    Vector2 ballPosition = info.getBall().value()->getPos();
    if(info.getRobot()->get()->getId() != GameStateManager::getCurrentGameState().keeperId && FieldComputations::pointIsInDefenseArea(info.getField().value(), ballPosition)){
        ballPosition = control::ControlUtils::projectPositionToOutsideDefenseArea(info.getField().value(), ballPosition, control_constants::AVOID_BALL_DISTANCE);
    }

    // the robot will go to the position of the ball
    double ballDistance = (ballPosition - robotPosition).length();
    Vector2 newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance - control_constants::CENTER_TO_FRONT + 0.035) ;

    if (ballDistance < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setAngle((ballPosition - robotPosition).angle());
        skillStpInfo.setDribblerSpeed(35);
    }

    skillStpInfo.setPositionToMoveTo(newRobotPosition);

    return skillStpInfo;
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
