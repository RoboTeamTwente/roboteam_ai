//
// Created by ratoone on 18-11-19.
//

#ifndef RTT_POSITIONCONTROL_H
#define RTT_POSITIONCONTROL_H


#include <control/RobotCommand.h>
#include "VoronoiPathPlanning.h"
#include "BasicPathTracking.h"
#include <world/Robot.h>
#include "rtt_traits.h"
#include "interface/api/Input.h"


using namespace rtt;

template <typename PathP = VoronoiPathPlanning, typename PathT = BasicPathTracking>
class PositionControl {
private:

    static_assert(rtt::ai::position::type_traits::has_path_computation<PathP>::value,
            "Invalid PathPlanning type in PositionControl");
    static_assert(rtt::ai::position::type_traits::has_path_tracking<PathT>::value,
            "Invalid PathTracking type in PositionControl");

    PathP pathPlanningAlgorithm;
    PathT pathTrackingAlgorithm;

    std::map<int,std::list<Vector2>> computedPaths;

public:
    PositionControl() = default;

    PositionControl(double fieldWidth, double fieldLength, const std::vector<rtt::Vector2 *> &robotPositions)
        : pathPlanningAlgorithm{ PathP(fieldWidth, fieldLength, robotPositions) }
    {}

    RobotCommand computeAndTrackPath(const std::vector<std::shared_ptr<ai::world::Robot>>&robots, int robotId,
                                     const Vector2 &currentPosition,
                                     const Vector2 &currentVelocity, const Vector2 &targetPosition) {
        if (computedPaths.find(robotId) == computedPaths.end()) {
            computedPaths.insert({robotId, std::list<Vector2>()});
        }

        if (computedPaths[robotId].empty() || computedPaths[robotId].back() != targetPosition) {
            std::vector<Vector2*> robotPositions(robots.size());
            std::transform(robots.begin(), robots.end(), robotPositions.begin(),
                           [](auto robot)-> Vector2* {return &(robot->pos);});
            pathPlanningAlgorithm.setRobotPositions(robotPositions);
            computedPaths[robotId] = pathPlanningAlgorithm.computePath(currentPosition, targetPosition);
        }

        RobotCommand command = RobotCommand();
        double angle{ 0.0 };
        command.pos = computedPaths[robotId].front();
        pathTrackingAlgorithm.trackPath(
                currentPosition,
                currentVelocity,
                computedPaths[robotId],
                command.vel,
                angle);
        command.angle = angle;

        for (const auto& point: computedPaths[robotId]) {
            rtt::ai::interface::Input::drawData(rtt::ai::interface::Visual::PATHFINDING_DEBUG, {point}, Qt::green,
                                                robotId,
                                                rtt::ai::interface::Drawing::DOTS, 12, 12);
            rtt::ai::interface::Input::drawData(rtt::ai::interface::Visual::PATHFINDING_DEBUG, {point, point + command.vel * 0.4},
                                       Qt::red,
                                       robotId,
                                       rtt::ai::interface::Drawing::LINES_CONNECTED);
        }
        return command;
    }
};


#endif //RTT_POSITIONCONTROL_H
