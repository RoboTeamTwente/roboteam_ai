//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/PidTracking.h"

namespace rtt::ai::control {

Position PidTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId, double angle, stp::PIDType pidType) {
    PositionControlUtils::removeFirstIfReached(pathPoints, currentPosition);
    if (pidMapping.find(robotId) == pidMapping.end()) {
        pidMapping[robotId] = std::make_pair(PID(), PID());
        pidMapping[robotId].first.setMaxIOutput(MAX_VELOCITY);
        pidMapping[robotId].second.setMaxIOutput(MAX_VELOCITY);
    }

    updatePIDValues(pidType, robotId);

    Vector2 velocity{};
    velocity.x = pidMapping[robotId].first.getOutput(currentPosition.x, pathPoints.front().x);
    velocity.y = pidMapping[robotId].second.getOutput(currentPosition.y, pathPoints.front().y);

    return {velocity, angle};
}

void PidTracking::updatePIDValues(stp::PIDType pidType, int robotID) {
    std::tuple<double, double, double> newPID;

    switch (pidType) {
        case stp::PIDType::DEFAULT: {
            newPID = interface::Output::getNumTreePid();
            break;
        }
        case stp::PIDType::RECEIVE: {
            newPID = interface::Output::getReceivePid();
            break;
        }
        case stp::PIDType::INTERCEPT: {
            newPID = interface::Output::getInterceptPid();
        }
        case stp::PIDType::KEEPER: {
            newPID = interface::Output::getKeeperPid();
            break;
        }
        case stp::PIDType::KEEPER_INTERCEPT: {
            newPID = interface::Output::getKeeperInterceptPid();
            break;
        }
    }

    pidMapping[robotID].first.setPID(newPID);
    pidMapping[robotID].second.setPID(newPID);
}
}  // namespace rtt::ai::control