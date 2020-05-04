//
// Created by baris on 16/11/18.
//

#ifndef ROBOTEAM_AI_CONTROLUTILS_H
#define ROBOTEAM_AI_CONTROLUTILS_H

#include <roboteam_utils/Arc.h>
#include <roboteam_utils/Line.h>
#include <cmath>
#include <optional>
#include "utilities/Constants.h"
#include "world/Field.h"
#include "world/FieldComputations.h"
#include "world_new/views/WorldDataView.hpp"

using Vector2 = rtt::Vector2;
using Angle = rtt::Angle;

namespace rtt::ai {

// fwd declarations
namespace world {
class WorldData;
class Robot;
}  // namespace world

    namespace control {
        using namespace rtt::ai::world;

        class ControlUtils {
        public:
            static Vector2 calculateForce(const rtt::Vector2 &vector, double weight, double minDistance);

            static Vector2
            velocityLimiter(const Vector2 &vel, double maxVel = Constants::MAX_VEL(), double minVel = 0.0,
                            bool listenToReferee = true);

            static Vector2
            accelerationLimiter(const Vector2 &targetVel, const Vector2 &prevVel, const Angle &targetAngle,
                                double sidewaysAcceleration = Constants::MAX_ACC_LOWER() / Constants::TICK_RATE(),
                                double forwardsAcceleration = Constants::MAX_ACC_UPPER() / Constants::TICK_RATE(),
                                double sidewaysDeceleration = Constants::MAX_DEC_LOWER() / Constants::TICK_RATE(),
                                double forwardsDeceleration = Constants::MAX_DEC_UPPER() / Constants::TICK_RATE());

            static bool
            objectVelocityAimedToPoint(const Vector2 &objectPosition, const Vector2 &velocity, const Vector2 &point,
                                       double maxDifference = 0.3);
        };

    }  // namespace control
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_CONTROLUTILS_H
