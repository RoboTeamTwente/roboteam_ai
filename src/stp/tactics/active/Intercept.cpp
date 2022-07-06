//
// Created by timovdk on 3/18/20.
//

#include "stp/tactics/active/Intercept.h"

#include "roboteam_utils/Line.h"
#include "stp/skills/GoToPos.h"
#include "world/FieldComputations.h"
#include "stp/computations/TimingComputations.h"
#include "roboteam_utils/Print.h"

namespace rtt::ai::stp::tactic {

Intercept::Intercept() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> Intercept::calculateInfoForSkill(const StpInfo& info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Rotate robot towards the ball
    skillStpInfo.setAngle(calculateAngle(info.getRobot().value(), info.getBall().value()));

    skillStpInfo.setPositionToMoveTo(calculateInterceptPosition(info));

    return skillStpInfo;
}

Vector2 Intercept::calculateInterceptPosition(const StpInfo& info) {
    auto ball = info.getBall().value();
    auto robot = info.getRobot().value();

    if (ball->getVelocity().length() < control_constants::BALL_STILL_VEL) return info.getRobot()->get()->getPos();

    auto ballTrajectory = LineSegment(ball->getPos(), ball->getPos() + ball->getVelocity().stretchToLength(info.getField()->getFieldLength()));

    auto distToBallTrajectory = ballTrajectory.distanceToLine(robot->getPos());
    if (distToBallTrajectory < 0.10)  return ballTrajectory.project(robot->getPos()); // If we're already standing on the line, stay there

    if (ballIsAccelerating(ball->getVelocity().length())){
        return FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), ball->getPos() + ball->getVelocity().stretchToLength(5), ballTrajectory.start, ballTrajectory.end);
    }

    auto interceptDistance = ball->getVelocity().length() * ballTrajectory.distanceToLine(robot->getPos());
    auto interceptPos = ball->getPos() + ball->getVelocity().stretchToLength(interceptDistance);

    // return interceptPos;
    return FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), interceptPos, ballTrajectory.start, ballTrajectory.end);
}

bool Intercept::ballIsAccelerating(double ballVelocity) {
    bool isAccelerating = ballVelocity > previousBallVelocity;
    previousBallVelocity = ballVelocity;
    return isAccelerating;
}

double Intercept::calculateAngle(const world::view::RobotView& robot, const world::view::BallView& ball) { return (ball->getPos() - robot->getPos()).angle(); }

bool Intercept::isTacticFailing(const StpInfo& info) noexcept {
    return false;
}

bool Intercept::shouldTacticReset(const StpInfo& info) noexcept {
    return false;
}

bool Intercept::isEndTactic() noexcept {
    // This is an end tactic
    return true;
}

const char* Intercept::getName() { return "Intercept"; }

bool Intercept::forceTacticSuccess(const StpInfo& info) noexcept {
    return (info.getBall()->get()->getVelocity().length() < control_constants::BALL_STILL_VEL);
}

}  // namespace rtt::ai::stp::tactic