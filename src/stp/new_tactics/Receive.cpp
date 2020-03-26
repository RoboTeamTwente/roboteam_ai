//
// Created by jordi on 13-03-20.
//

#include "include/roboteam_ai/stp/new_tactics/Receive.h"
#include "include/roboteam_ai/stp/new_skills/GoToPos.h"
#include "include/roboteam_ai/stp/new_skills/Rotate.h"

namespace rtt::ai::stp::tactic {

Receive::Receive() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos(), skill::Rotate()};
    skills.initialize();
}

void Receive::onInitialize() noexcept {}

void Receive::onUpdate(Status const &status) noexcept {}

void Receive::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo Receive::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    // Rotate robot towards the ball
    skillStpInfo.setAngle(calculateAngle(info.getRobot().value(), info.getBall().value()));

    // If ball is close to robot, turn on dribbler
    skillStpInfo.setDribblerSpeed(determineDribblerSpeed(info.getRobot().value()));

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

double Receive::calculateAngle(const world_new::view::RobotView &robot, const world_new::view::BallView &ball) { return (ball->getPos() - robot->getPos()).angle(); }

int Receive::determineDribblerSpeed(const world_new::view::RobotView &robot) {
    double turnOnDribblerDistance = 1.0;
    return robot->getDistanceToBall() < turnOnDribblerDistance ? 100 : 0;
}

}  // namespace rtt::ai::stp::tactic