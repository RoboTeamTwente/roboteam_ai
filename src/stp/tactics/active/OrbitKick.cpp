//
// Created by tijmen on 01-07-22.
//

#include "stp/tactics/active/OrbitKick.h"

#include "control/ControlUtils.h"
#include "stp/skills/Kick.h"
#include "stp/skills/OrbitAngular.h"

namespace rtt::ai::stp::tactic {

OrbitKick::OrbitKick() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::OrbitAngular(), skill::Kick()};
}

std::optional<StpInfo> OrbitKick::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    if (!skillStpInfo.getPositionToShootAt() || !skillStpInfo.getRobot() || !skillStpInfo.getBall()) return std::nullopt;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPositionToShootAt().value() - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the kick force
    double distanceBallToTarget = (info.getBall()->get()->position - info.getPositionToShootAt().value()).length();
    skillStpInfo.setKickChipVelocity(control::ControlUtils::determineKickForce(distanceBallToTarget, skillStpInfo.getShotType()));

    skillStpInfo.setDribblerSpeed(100);

    return skillStpInfo;
}

bool OrbitKick::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool OrbitKick::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if the robot doesn't have the ball or if there is no shootTarget
    return !info.getPositionToShootAt() || !info.getRobot()->get()->hasBall();
}

bool OrbitKick::shouldTacticReset(const StpInfo &info) noexcept {
    return false;
}

const char *OrbitKick::getName() { return "Orbit Kick"; }

}  // namespace rtt::ai::stp::tactic
