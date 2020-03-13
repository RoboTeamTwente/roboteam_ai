//
// Created by timovdk on 3/13/20.
//

#include "stp/new_tactics/ChipAtPos.h"

#include <stp/new_skills/Chip.h>
#include <stp/new_skills/Rotate.h>

namespace rtt::ai::stp::tactic {

ChipAtPos::ChipAtPos() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Chip()};
    skills.initialize();
}

void ChipAtPos::onInitialize() noexcept {}

void ChipAtPos::onUpdate(Status const &status) noexcept {}

void ChipAtPos::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo ChipAtPos::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    // Calculate the angle the robot needs to aim
    double angleToBall = (info.getTargetPos().second - info.getBall()->get()->getPos()).angle();
    skillStpInfo.setAngle(angleToBall);

    // Calculate the distance and the chip force
    double distanceBallToTarget = (info.getBall()->get()->getPos() - info.getTargetPos().second).length();
    skillStpInfo.setKickChipVelocity(determineChipForce(distanceBallToTarget, skillStpInfo.getKickChipType()));

    // When chipping, we don't need to activate the dribbler
    skillStpInfo.setDribblerSpeed(0);

    return skillStpInfo;
}

/// Determine how fast we should chip for a pass at a given distance
double ChipAtPos::determineChipForce(double distance, KickChipType desiredBallSpeedType) noexcept {
    const double maxPowerDist = rtt::ai::Constants::MAX_POWER_KICK_DISTANCE();

    double velocity = 0;
    switch (desiredBallSpeedType) {
        case DRIBBLE_KICK: {
            velocity = sqrt(distance) * rtt::ai::Constants::MAX_KICK_POWER() / (sqrt(maxPowerDist) * 1.5);
            break;
        }
        case BALL_PLACEMENT: {
            if (distance > 2.5) {
                velocity = Constants::GRSIM() ? 6.01 : 2.01;
            } else {
                velocity = Constants::GRSIM() ? 3.01 : 1.01;
            }
            break;
        }
        case PASS: {
            if (distance >= maxPowerDist) {
                velocity = Constants::MAX_KICK_POWER();
            } else if (Constants::GRSIM()) {
                velocity = std::min(1.4 * distance / maxPowerDist * Constants::MAX_KICK_POWER(), Constants::DEFAULT_KICK_POWER());
            } else {
                velocity = std::min(distance / maxPowerDist * Constants::MAX_KICK_POWER(), Constants::DEFAULT_KICK_POWER() * 0.7);
            }
            break;
        }
        case MAX_SPEED: {
            velocity = rtt::ai::Constants::MAX_KICK_POWER();
            break;
        }
        default: {
            velocity = rtt::ai::Constants::MAX_KICK_POWER();
        }
    }

    // limit the output to the max chip speed
    return std::min(std::max(velocity, 1.01), rtt::ai::Constants::MAX_KICK_POWER());
}

bool ChipAtPos::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool ChipAtPos::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if the robot doesn't have the ball or if the targetPosType is not a shootTarget
    return !info.getRobot()->hasBall() || info.getTargetPos().first != SHOOTTARGET;
}

bool ChipAtPos::shouldTacticReset(const StpInfo &info) noexcept {
    // Never reset tactic
    return false;
}
}  // namespace rtt::ai::stp::tactic