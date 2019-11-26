//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H


#include <control/RobotCommand.h>
#include "VoronoiPathPlanning.h"
#include "BasicPathTracking.h"
#include <world/Robot.h>

using namespace rtt;

class PositionControl {
private:
    VoronoiPathPlanning pathPlanningAlgorithm;
    BasicPathTracking pathTrackingAlgorithm;

    std::map<int,std::list<Vector2>> computedPaths;

public:
    PositionControl() = default;

    PositionControl(double fieldWidth, double fieldLength, const std::vector<rtt::Vector2 *> &robotPositions);

    RobotCommand computeAndTrackPath(const std::vector<std::shared_ptr<ai::world::Robot>>&robots, int robotId,
                                     const Vector2 &currentPosition,
                                     const Vector2 &currentVelocity, const Vector2 &targetPosition);
};


#endif //RTT_POSITIONCONTROL_H
