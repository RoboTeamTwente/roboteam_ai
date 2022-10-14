//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include <roboteam_utils/Arc.h>
#include <roboteam_utils/Grid.h>
#include <roboteam_utils/Line.h>
#include <utilities/StpInfoEnums.h>

#include <cmath>
#include <optional>

#include "utilities/Constants.h"
#include "world/Field.h"

using Vector2 = rtt::Vector2;
using Angle = rtt::Angle;

namespace rtt::ai::control {

class ControlUtils {
   public:
    static Vector2 calculateForce(const rtt::Vector2 &vector, double weight, double minDistance);

    static Vector2 velocityLimiter(const Vector2 &vel, double maxVel = Constants::MAX_VEL(), double minVel = 0.0, bool listenToReferee = true);

    static Vector2 accelerationLimiter(const Vector2 &targetVel, const Vector2 &prevVel, const Angle &targetAngle,
                                       double sidewaysAcceleration = Constants::MAX_ACC_LOWER() / Constants::STP_TICK_RATE(),
                                       double forwardsAcceleration = Constants::MAX_ACC_UPPER() / Constants::STP_TICK_RATE(),
                                       double sidewaysDeceleration = Constants::MAX_DEC_LOWER() / Constants::STP_TICK_RATE(),
                                       double forwardsDeceleration = Constants::MAX_DEC_UPPER() / Constants::STP_TICK_RATE());

    static bool objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity, const Vector2 &point, double maxDifference = 0.3);

    /**
     * Determines the kick force based on the distance and the type of kick
     * @param distance distance to the target
     * @param shotType type of the kick
     * @return a kick speed between min and max kick speed
     */
    static double determineKickForce(const double distance, stp::ShotType shotType) noexcept;

    /**
     * Determine the max allowed velocity considering the game state and whether the robot has the ball
     * @param hasBall Whether this robot has the ball
     * @return The max allowed velocity for this robot
     */
    static double getMaxVelocity(bool hasBall);
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_CONTROLUTILS_H
