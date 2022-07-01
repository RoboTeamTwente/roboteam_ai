//
// Created by ratoone on 10-03-20.
/// Moves the robot to the BALL in a straight line and ROTATES the robot towards to ball.

/// ACTIVE
//

#include "stp/tactics/active/GetBall.h"

#include "control/ControlUtils.h"
#include "stp/skills/GoToPos.h"
#include "stp/skills/Rotate.h"
#include "world/FieldComputations.h"
#include "roboteam_utils/Print.h"
namespace rtt::ai::stp::tactic {
GetBall::GetBall() { skills = collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()}; }

std::optional<StpInfo> GetBall::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    Vector2 robotPosition = skillStpInfo.getRobot().value()->getPos();
    Vector2 ballPosition = skillStpInfo.getBall().value()->position;
    double ballDistance = (ballPosition - robotPosition).length();

    if (skillStpInfo.getRobot()->get()->getAngleDiffToBall() > Constants::HAS_BALL_ANGLE() && ballDistance < control_constants::AVOID_BALL_DISTANCE) {
        RTT_DEBUG("Not at right angle to get ball angle = ", skillStpInfo.getRobot()->get()->getAngleDiffToBall());
        // don't move too close to the ball until the angle to the ball is (roughly) correct
        skillStpInfo.setPositionToMoveTo(
            FieldComputations::projectPointToValidPosition(info.getField().value(), skillStpInfo.getRobot()->get()->getPos(), info.getObjectsToAvoid()));
    } else {
        auto getBallDistance =  std::max(ballDistance - control_constants::CENTER_TO_FRONT, 0.1);
        Vector2 newRobotPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(getBallDistance);
        newRobotPosition = FieldComputations::projectPointToValidPosition(info.getField().value(), newRobotPosition, info.getObjectsToAvoid());
        skillStpInfo.setPositionToMoveTo(newRobotPosition);
    }

    skillStpInfo.setAngle((ballPosition - robotPosition).angle());

    if (ballDistance < control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerSpeed(100);
    }

    if (info.getRobot().value()->hasBall()) hasBallCounter++;
    else if (hasBallCounter > 0) hasBallCounter--;

    return skillStpInfo;
}

bool GetBall::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool GetBall::shouldTacticReset(const StpInfo &info) noexcept {
    if (!info.getRobot()->get()->hasBall()){
        RTT_DEBUG("Resetting get ball");
        return true;
    }
    return false;
    if (hasBallCounter <= 0 && skills.current_num() == 1) RTT_DEBUG("Resetting getBall!");
    return hasBallCounter <= 0 && skills.current_num() == 1;
}

bool GetBall::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool GetBall::forceTacticSuccess(const StpInfo &info) noexcept {
    return info.getRobot()->get()->hasBall();
    if (hasBallCounter > 10) RTT_DEBUG("GetBall is success!")
    return hasBallCounter > 10; }

const char *GetBall::getName() { return "Get Ball"; }

}  // namespace rtt::ai::stp::tactic
