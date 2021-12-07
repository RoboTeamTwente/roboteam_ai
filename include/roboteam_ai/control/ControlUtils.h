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
                                       double sidewaysAcceleration = Constants::MAX_ACC_LOWER() / Constants::TICK_RATE(),
                                       double forwardsAcceleration = Constants::MAX_ACC_UPPER() / Constants::TICK_RATE(),
                                       double sidewaysDeceleration = Constants::MAX_DEC_LOWER() / Constants::TICK_RATE(),
                                       double forwardsDeceleration = Constants::MAX_DEC_UPPER() / Constants::TICK_RATE());

    static bool objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity, const Vector2 &point, double maxDifference = 0.3);


    /**
     * Project given position to within the field with a certain margin
     * @param field The field class used to determine where the field lines are
     * @param position The position to be projected to within the field
     * @param margin The margin that should be used when calculating the new position. The position will have a minimum of this distance to the field lines
     * @return The position projected to within the field
     */
    static Vector2 projectPositionToWithinField(const rtt_world::Field &field, Vector2 position, double margin);

    /**
     * Project given position to within the field with a certain margin
     * @param field The field class used to determine where the defense area is
     * @param position The position to be projected to outside of the defense area
     * @param margin The margin that should be used when calculating the new position. The position will have a minimum of this distance to the defense area
     * @return The position projected to outside of the defense area
     */
    static Vector2 projectPositionToOutsideDefenseArea(const rtt_world::Field &field, Vector2 position, double margin);

    /**
     * Project given position to a valid position (within the field and outside of the defense area)
     * @param field The field class used to determine where the field lines and defense area are
     * @param position The position to be projected to within the field
     * @param id The id of the robot for this position, used to check whether this robot is the keeper. If so, it is allowed within our defense area
     * @param margin The margin that should be used when calculating the new position. The position will have a minimum of this distance to the field lines and defense area
     * @return The position projected to within the field and outside the defense areas
     */
    static Vector2 projectPointToValidPosition(const rtt::world::Field &field, Vector2 position, int id, double margin);

    /**
     * Determines the chip force based on the distance and the type of chip
     * @param distance distance to the target
     * @param shotType type of the chip
     * @return a chip speed between min and max chip speed
     */
    static double determineChipForce(const double distance, stp::ShotType shotType) noexcept;

    /**
     * Determines the kick force based on the distance and the type of kick
     * @param distance distance to the target
     * @param shotType type of the kick
     * @return a kick speed between min and max kick speed
     */
    static double determineKickForce(const double distance, stp::ShotType shotType) noexcept;
};

}  // namespace rtt::ai::control

#endif  // ROBOTEAM_AI_CONTROLUTILS_H
