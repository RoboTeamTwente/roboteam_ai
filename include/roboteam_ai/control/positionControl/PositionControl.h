//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H


#include "control/RobotCommand.h"
#include "control/positionControl/pathPlanning/NumTreesPlanning.h"
#include "control/positionControl/pathTracking/BasicPathTracking.h"
#include "CollisionDetector.h"
#include <world_new/Robot.hpp>


namespace rtt::ai::control {

class PositionControl {
private:

    const double MAX_DEVIATION = 0.3;

    NumTreesPlanning *pathPlanningAlgorithm = nullptr;
    BasicPathTracking *pathTrackingAlgorithm = nullptr;
    CollisionDetector *collisionDetector = nullptr;

    const std::vector<rtt::world_new::robot::Robot> &robots;

    std::unordered_map<int, std::list<Vector2>> computedPaths;

public:
    explicit PositionControl(const std::vector<rtt::world_new::robot::Robot> &robots);

    RobotCommand computeAndTrackPath(world::Field *field, int robotId, const Vector2 &currentPosition,
                                     const Vector2 &currentVelocity, const Vector2 &targetPosition);

    /**
     * The computed path should be recalculated if: <br>
     * - it is empty (no path yet) <br>
     * - the target position changed with at least MAX_DEVIATION <br>
     * - the robot will collide with another one by the next path point
     * @param targetPos final target position
     * @param robotId the ID of the current robot
     * @return true if one of the above conditions are true, false otherwise
     */
    bool shouldRecalculatePath(const Vector2 &currentPosition, const Vector2 &targetPos, int robotId);
};

}
#endif //RTT_POSITIONCONTROL_H
