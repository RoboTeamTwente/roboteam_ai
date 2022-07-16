//
// Created by baris on 16/11/18.
//

#include "control/ControlUtils.h"

#include <roboteam_utils/Grid.h>

#include "stp/constants/ControlConstants.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/StpInfoEnums.h"
#include "world/Field.h"
#include "world/World.hpp"

namespace rtt::ai::control {

double ControlUtils::getMaxVelocity(bool hasBall) {
    double maxVel = rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
    if (hasBall) maxVel = std::min(stp::control_constants::MAX_VEL_WHEN_HAS_BALL, maxVel);
    return maxVel;
}
/// Limits velocity to maximum velocity. it defaults to the max velocity stored in Referee.
Vector2 ControlUtils::velocityLimiter(const Vector2 &vel, double maxVel, double minVel, bool listenToReferee) {
    if (listenToReferee) {
        double refereeMaxVel = rtt::ai::GameStateManager::getCurrentGameState().getRuleSet().maxRobotVel;
        if (refereeMaxVel < maxVel) {
            maxVel = refereeMaxVel;
        }
    }

    if (vel.length() > maxVel) {
        return vel.stretchToLength(maxVel);
    } else if (vel.length() < minVel) {
        return vel.stretchToLength(minVel);
    }
    return vel;
}

/// Limits acceleration
Vector2 ControlUtils::accelerationLimiter(const Vector2 &targetVel, const Vector2 &prevVel, const Angle &targetAngle, double sidewaysAcceleration, double forwardsAcceleration,
                                          double sidewaysDeceleration, double forwardsDeceleration) {
    Vector2 deltaVel = targetVel - prevVel;

    // calculate if the robot is driving forwards or sideways
    Angle robotAngleDifference = targetVel.toAngle() - targetAngle;
    Vector2 robotVectorDifference = robotAngleDifference.toVector2();
    double a = fabs(robotVectorDifference.x);
    auto acceleration = sidewaysAcceleration * (1 - a) + forwardsAcceleration * a;
    auto deceleration = sidewaysDeceleration * (1 - a) + forwardsDeceleration * a;
    // a = 0 -> sideways
    // a = 1 -> forwards

    // calculate if the robot is accelerating or decelerating
    Angle accelerationAngleDifference = deltaVel.toAngle() - targetVel.toAngle();
    double b = fabs(accelerationAngleDifference) * M_1_PI;
    auto finalAcceleration = acceleration * (1 - b) + deceleration * b;
    // b = 0 -> acceleration
    // b = 1 -> deceleration

    if (deltaVel.length() < finalAcceleration) {
        return targetVel;
    }
    return prevVel + deltaVel.stretchToLength(finalAcceleration);
}

/// Calculate the force of a given vector + a certain type.
/// the basic formula is: force = weight/distance^2 * unit vector
Vector2 ControlUtils::calculateForce(const Vector2 &vector, double weight, double minDistance) {
    // if the object is close enough, it's forces should affect. Otherwise don't change anything.
    if (vector.length() < minDistance && vector.length2() > 0) {
        return vector.normalize() * (weight / vector.length2());
    }
    return {0, 0};
}

bool ControlUtils::objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity, const Vector2 &point, double maxDifference) {
    double exactAngleTowardsPoint = (point - objectPosition).angle();

    // Note: The angles should NOT be constrained here. This is necessary.
    return (velocity.length() > 0 && velocity.angle() > exactAngleTowardsPoint - maxDifference / 2 && velocity.angle() < exactAngleTowardsPoint + maxDifference / 2);
}

/// Calculates the chip force
double ControlUtils::determineChipForce(const double distance, stp::ShotType shotType) noexcept {
    // TODO: Needs further tuning
    constexpr double TARGET_FACTOR{0.5};
    constexpr double PASS_FACTOR{0.745};

    if (shotType == stp::ShotType::MAX) return stp::control_constants::MAX_CHIP_POWER;

    double limitingFactor{};
    // Pick the right limiting factor based on ballSpeedType and whether we use GRSIM or not
    if (shotType == stp::ShotType::PASS) {
        limitingFactor = PASS_FACTOR;
    } else if (shotType == stp::ShotType::TARGET) {
        limitingFactor = TARGET_FACTOR;
    } else {
        RTT_ERROR("No valid ballSpeedType, kick velocity set to 0")
        return 0;
    }

    // Calculate the velocity based on this function with the previously set limitingFactor
    auto velocity = distance * limitingFactor;

    // Make sure velocity is always between MIN_CHIP_POWER and MAX_CHIP_POWER
    return std::clamp(velocity, stp::control_constants::MIN_CHIP_POWER, stp::control_constants::MAX_CHIP_POWER);
}

/// Calculate the kick force
double ControlUtils::determineKickForce(const double distance, stp::ShotType shotType) noexcept {
    if (shotType == stp::ShotType::MAX) return stp::control_constants::MAX_KICK_POWER;

    double kickForce;
    if (shotType == stp::ShotType::PASS) {
        kickForce = 1.5;
    } else if (shotType == stp::ShotType::TARGET) {
        kickForce = 0.60;
    } else {
        RTT_WARNING("No shotType set! Setting kickForce to 0")
        kickForce = 0;
    }
    // Calculate the velocity based on this function with the previously set limitingFactor
    auto velocity = distance * kickForce;

    auto maxKickPower = shotType == stp::ShotType::PASS ? stp::control_constants::MIN_KICK_POWER_PASSING : stp::control_constants::MIN_KICK_POWER;
    // Make sure velocity is always between MIN_KICK_POWER and MAX_KICK_POWER
    return std::clamp(velocity, stp::control_constants::MIN_KICK_POWER, maxKickPower);
}

}  // namespace rtt::ai::control
