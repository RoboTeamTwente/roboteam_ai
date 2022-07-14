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

    if (ball->velocity.length() < control_constants::BALL_IS_MOVING_SLOW_LIMIT) return info.getBall()->get()->expectedEndPosition;

    auto ballTrajectory = LineSegment(ball->position, ball->position + ball->velocity.stretchToLength(info.getField()->getFieldLength()));

    auto distToBallTrajectory = ballTrajectory.distanceToLine(robot->getPos());
    if (distToBallTrajectory < 0.10)
        return FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), ballTrajectory.start, ballTrajectory.start, ballTrajectory.end);

    if (ballIsAccelerating(ball->velocity.length())){
        return FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), ball->expectedEndPosition, ballTrajectory.start, ballTrajectory.end);
    }

    auto interceptDistance = ball->velocity.length() * ballTrajectory.distanceToLine(robot->getPos());
    auto interceptPos = ball->position + ball->velocity.stretchToLength(interceptDistance);

    // return interceptPos;
    return FieldComputations::projectPointToValidPositionOnLine(info.getField().value(), interceptPos, ballTrajectory.start, ballTrajectory.end);
}

bool Intercept::ballIsAccelerating(double ballVelocity) {
    bool isAccelerating = ballVelocity > previousBallVelocity * 1.2; // Due to filtering, the ball velocity will "slowly" go up, this checks if this is the case
    previousBallVelocity = ballVelocity;
    return isAccelerating;
}

double Intercept::calculateAngle(const world::view::RobotView& robot, const world::view::BallView& ball) { return (ball->position - robot->getPos()).angle(); }

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
    return (info.getBall()->get()->velocity.length() < control_constants::BALL_STILL_VEL);
}

}  // namespace rtt::ai::stp::tactic