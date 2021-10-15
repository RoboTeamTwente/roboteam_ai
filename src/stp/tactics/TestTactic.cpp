//
// Created by roboteam on 9/3/20.
//

#include "stp/tactics/TestTactic.h"

#include "stp/skills/TestSkill.h"

#include "stp/skills/GoToPos.h"

#include "roboteam_utils/Print.h"

namespace rtt::ai::stp {

TestTactic::TestTactic() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::GoToPos()};
}

std::optional<StpInfo> TestTactic::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return std::nullopt;
    skillStpInfo.setDribblerSpeed(100);
    if (skillStpInfo.getRobot() && skillStpInfo.getBall()) {
        auto ballPosition = info.getBall().value()->getPos();
        auto robotPosition = info.getRobot()->get()->getPos();
        auto ballDistance = (ballPosition - robotPosition).length();
        auto targetPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance - control_constants::CENTER_TO_FRONT + 0.03);
        skillStpInfo.setPositionToMoveTo(targetPosition);
        auto targetAngle = (ballPosition - robotPosition).angle();
        skillStpInfo.setAngle(targetAngle);
    }
    return skillStpInfo;
}

[[maybe_unused]] std::optional<StpInfo> TestTactic::GoToBall(StpInfo const &info){
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getRobot() || !skillStpInfo.getBall()){
        skillStpInfo.setPositionToMoveTo(Vector2(0,0));
        return skillStpInfo;
    }
    auto ballPosition = info.getBall().value()->getPos();
    auto robotPosition = info.getRobot()->get()->getPos();
    auto ballDistance = (ballPosition - robotPosition).length();
    auto targetPosition = robotPosition + (ballPosition - robotPosition).stretchToLength(ballDistance - control_constants::CENTER_TO_FRONT + 0.03);
    skillStpInfo.setPositionToMoveTo(targetPosition);
    return skillStpInfo;
}

[[maybe_unused]] std::optional<StpInfo> TestTactic::RotateToBall(StpInfo const &info){
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return skillStpInfo;
    auto ballPosition = info.getBall().value()->getPos();
    auto robotPosition = info.getRobot()->get()->getPos();
    auto targetAngle = (ballPosition - robotPosition).angle();
    skillStpInfo.setAngle(targetAngle);
    return skillStpInfo;
}

[[maybe_unused]] std::optional<StpInfo> TestTactic::TurnOnDribbler(const StpInfo &info, double dribblerSpeed){
    StpInfo skillStpInfo = info;
    if (!skillStpInfo.getField()) return skillStpInfo;
    skillStpInfo.setDribblerSpeed(dribblerSpeed);
    return skillStpInfo;
}

bool TestTactic::isTacticFailing(const StpInfo &info) noexcept { return false; }

bool TestTactic::shouldTacticReset(const StpInfo &info) noexcept {
    return info.getRobot()->hasBall();
}

bool TestTactic::forceTacticSuccess(const StpInfo &info) noexcept { return info.getRobot()->hasBall(); }

bool TestTactic::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

const char *TestTactic::getName() { return "Test Tactic"; }

}  // namespace rtt::ai::stp
