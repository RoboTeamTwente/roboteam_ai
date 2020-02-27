//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H

#include "CollisionDetector.h"
#include "control/RobotCommand.h"
#include "control/positionControl/pathPlanning/NumTreesPlanning.h"
#include "control/positionControl/pathTracking/BasicPathTracking.h"
#include "control/positionControl/pathTracking/NumTreesTracking.h"
#include "world_new/views/RobotView.hpp"

namespace rtt::ai::control {

/**
 * The main position control class. Use this for your robot position control
 * requirements.
 */
class PositionControl {
   private:
    CollisionDetector collisionDetector;
    NumTreesPlanning pathPlanningAlgorithm = NumTreesPlanning(collisionDetector);
    NumTreesTracking pathTrackingAlgorithm;

    std::unordered_map<int, std::vector<Vector2>> computedPaths;

   public:
    /**
     * Generates a path according to the selected planning algorithm,
     * and tracks it using the selected tracking algorithm. In the case a collision
     * is detected (using the collision detector), the path is recalculated.
     * @param field the field object, used onwards by the collision detector
     * @param robotId the ID of the robot for which the path is calculated
     * @param currentPosition the current position of the aforementioned robot
     * @param currentVelocity its velocity
     * @param targetPosition the desired position that the robot has to reach
     * @return a RobotCommand, which can be fed directly in the output
     */
    RobotCommand computeAndTrackPath(const world::Field &field, int robotId, const Vector2 &currentPosition, const Vector2 &currentVelocity, const Vector2 &targetPosition);

    /**
     * Updates the robot view vector
     * @param robotPositions the RobotView vector of robots
     */
    void setRobotPositions(std::vector<Vector2> &robotPositions);

    /**
     * The computed path should be recalculated if: <br>
     * - it is empty (no path yet) <br>
     * - the target position changed with at least MAX_TARGET_DEVIATION <br>
     * - the robot will collide with another one by the next path point (ignored if the robot is not moving)
     * @param targetPos final target position
     * @param robotId the ID of the current robot
     * @return true if one of the above conditions are true, false otherwise
     */
    bool shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos,
                               const Vector2 &currentVelocity, int robotId);
};

}  // namespace rtt::ai::control
#endif  // RTT_POSITIONCONTROL_H
