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
            static double angleDifference(double A1, double A2);

            static int rotateDirection(double currentAngle, double targetAngle);

            static Vector2 calculateForce(const rtt::Vector2 &vector, double weight, double minDistance);

            static int lineOrientation(const Vector2 &p, const Vector2 &q, const Vector2 &r);

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

            static bool robotIsAimedAtPoint(int id, bool ourTeam, const Vector2 &point, const world_new::view::WorldDataView world, double maxDifference = 0.3);

            static bool clearLine(const Vector2 &fromPos, const Vector2 &toPos, const world_new::view::WorldDataView world, double safeDistanceFactor, bool includeKeeper);

            static Vector2 projectPositionToWithinField(const world::Field &field, Vector2 position, double margin);

            static Vector2 projectPositionToOutsideDefenseArea(const world::Field &field, Vector2 position, double margin);

            static const world_new::view::RobotView getRobotClosestToLine(std::vector<world_new::view::RobotView> robots, Vector2 const &lineStart, Vector2 const &lineEnd,
                bool lineWithEnds);

            static Vector2 getInterceptPointOnLegalPosition(const world::Field &field, Vector2 position, Line line, bool canMoveInDefenseArea, bool canMoveOutOfField,
                double defenseAreamargin, double outOfFieldMargin);
        };

    }  // namespace control
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_CONTROLUTILS_H
