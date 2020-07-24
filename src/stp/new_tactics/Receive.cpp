//
// Created by jordi on 13-03-20.
//

#include "stp/new_tactics/Receive.h"

#include "stp/new_skills/GoToPos.h"
#include "stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Receive::Receive() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
}

std::optional<StpInfo> Receive::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Rotate robot towards the ball
    skillStpInfo.setAngle(calculateAngle(info.getRobot().value(), info.getBall().value()));
    skillStpInfo.setPidType(PIDType::RECEIVE);

    // If ball is close to robot, turn on dribbler
    if (skillStpInfo.getRobot()->get()->getDistanceToBall() <= control_constants::TURN_ON_DRIBBLER_DISTANCE) {
        skillStpInfo.setDribblerSpeed(100);
    }

    return skillStpInfo;
}

bool Receive::isTacticFailing(const StpInfo &info) noexcept {
    // Receive tactic fails if targetType is not a receiveTarget
    return !info.getPositionToMoveTo();
}

bool Receive::shouldTacticReset(const StpInfo &info) noexcept {
    // Receive tactic resets when robot position is not close enough to the target position for receiving
    double errorMargin = stp::control_constants::GO_TO_POS_ERROR_MARGIN;
    return (info.getRobot().value()->getPos() - info.getPositionToMoveTo().value()).length() > errorMargin;
}

bool Receive::isEndTactic() noexcept {
    // Receive tactic is an end tactic
    return true;
}

double Receive::calculateAngle(const world::view::RobotView &robot, const world::view::BallView &ball) {
    if (robot->getDistanceToBall() <= 0.8) {
        return robot->getAngle();
    } else {
        return (ball->getPos() - robot->getPos()).angle();
    }
}

const char *Receive::getName() { return "Receive"; }

}  // namespace rtt::ai::stp::tactic