//
// Created by timovdk on 3/18/20.
/// TODO-Max the position should be determined within the tactic to Specify the tactic.
//

#include "stp/tactics/Intercept.h"

#include "stp/constants/ControlConstants.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Intercept::Intercept() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

bool Intercept::isEndTactic() noexcept {
    // This is an end tactic
    return true;
}

std::optional<StpInfo> Intercept::calculateInfoForSkill(const StpInfo& info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Rotate robot towards the ball
    skillStpInfo.setAngle(calculateAngle(info.getRobot().value(), info.getBall().value()));

    // If ball is close to robot, turn on dribbler
    skillStpInfo.setDribblerSpeed(determineDribblerSpeed(info.getRobot().value()));

    return skillStpInfo;
}

bool Intercept::isTacticFailing(const StpInfo& info) noexcept {
    // If the ball doesn't move, the robot can't intercept
    return info.getBall()->get()->velocity.length() <= stp::control_constants::BALL_STILL_VEL;
}

bool Intercept::shouldTacticReset(const StpInfo& info) noexcept {
    // If the robot does not have the ball, reset so GoToPos is called to move to the ball again
    return !info.getRobot().value()->hasBall();
}

double Intercept::calculateAngle(const world::view::RobotView& robot, const world::view::BallView& ball) { return (ball->position - robot->getPos()).angle(); }

int Intercept::determineDribblerSpeed(const world::view::RobotView& robot) {
    double turnOnDribblerDistance = 1.0;
    return robot->getDistanceToBall() < turnOnDribblerDistance ? 100 : 0;
}

const char* Intercept::getName() { return "Intercept"; }

}  // namespace rtt::ai::stp::tactic