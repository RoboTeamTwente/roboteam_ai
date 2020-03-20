//
// Created by timovdk on 3/12/20.
//

#include "stp/new_tactics/KickAtPos.h"

#include <roboteam_utils/Print.h>
#include <stp/new_skills/Kick.h>
#include <stp/new_skills/Rotate.h>

namespace rtt::ai::stp::tactic {

KickAtPos::KickAtPos() {
    // Create state machine of skills and initialize first skill
    skills = rtt::collections::state_machine<Skill, Status, StpInfo>{skill::Rotate(), skill::Kick()};
    skills.initialize();
}

void KickAtPos::onInitialize() noexcept {}

void KickAtPos::onUpdate(Status const &status) noexcept {}

void KickAtPos::onTerminate() noexcept {
    // Call terminate on all skills
    for (auto &x : skills) {
        x->terminate();
    }
}

StpInfo KickAtPos::calculateInfoForSkill(StpInfo const &info) noexcept {
    StpInfo skillStpInfo = info;

    // Calculate the angle the robot needs to aim
    double angleToTarget = (info.getPosition().second - info.getRobot().value()->getPos()).angle();
    skillStpInfo.setAngle(angleToTarget);

    // Calculate the distance and the kick force
    double distanceBallToTarget = (info.getBall()->get()->getPos() - info.getPosition().second).length();
    skillStpInfo.setKickChipVelocity(determineKickForce(distanceBallToTarget, skillStpInfo.getKickChipType()));

    // When the angle is not within the margin, dribble so we don't lose the ball while rotating
    double errorMargin = Constants::GOTOPOS_ANGLE_ERROR_MARGIN() * M_PI;
    if (fabs(info.getRobot()->get()->getAngle().shortestAngleDiff(angleToTarget)) >= errorMargin) {
        skillStpInfo.setDribblerSpeed(100);
    } else {
        skillStpInfo.setDribblerSpeed(0);
    }

    return skillStpInfo;
}

/// Determine how fast we should kick for a pass at a given distance
// TODO: This is bad code full of magic numbers so please refactor at a later stage :)
double KickAtPos::determineKickForce(double distance, KickChipType desiredBallSpeedType) noexcept {
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

    // limit the output to the max kick speed
    return std::min(std::max(velocity, 1.01), rtt::ai::Constants::MAX_KICK_POWER());
}

bool KickAtPos::isEndTactic() noexcept {
    // This is not an end tactic
    return false;
}

bool KickAtPos::isTacticFailing(const StpInfo &info) noexcept {
    // Fail tactic if:
    // robot doesn't have the ball && ball is still (to prevent chasing a ball that was just shot)
    // or if the targetPosType is not a shootTarget
    return (info.getBall()->get()->getVelocity().length() < Constants::BALL_STILL_VEL() && !info.getRobot()->hasBall(Constants::ROBOT_RADIUS() + (Constants::BALL_RADIUS() * 2))) || info.getPosition().first != SHOOT_TO_POSITION;
}

bool KickAtPos::shouldTacticReset(const StpInfo &info) noexcept {
    // Reset when angle is wrong outside of the rotate skill, reset to rotate again
    if(skills.current_num() != 0) {
        double errorMargin = Constants::GOTOPOS_ANGLE_ERROR_MARGIN() * M_PI;
        return fabs(info.getRobot().value()->getAngle().shortestAngleDiff(info.getAngle())) > errorMargin;
    }
    return false;
}
}  // namespace rtt::ai::stp::tactic
