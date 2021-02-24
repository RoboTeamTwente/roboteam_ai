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

void PidTracking::updatePidValuesFromInterface(bool isKeeper) {
    auto newPid = isKeeper ? Constants::standardKeeperPID() : Constants::standardNumTreePID();
    for (auto pid : pidMapping) {
        pidMapping[pid.first].first.setPID(newPid);
        pidMapping[pid.first].second.setPID(newPid);
    }
}

void PidTracking::updatePIDValues(stp::PIDType pidType, int robotID) {
    std::tuple<double, double, double> newPID;

    switch (pidType) {
      case stp::PIDType::DEFAULT: {
            newPID = Constants::standardNumTreePID();
            break;
        }
        case stp::PIDType::RECEIVE: {
            newPID = Constants::standardReceivePID();
            break;
        }
        case stp::PIDType::INTERCEPT: {
            newPID = Constants::standardInterceptPID();
        }
        case stp::PIDType::KEEPER: {
            newPID = Constants::standardKeeperPID();
            break;
        }
        case stp::PIDType::KEEPER_INTERCEPT: {
            newPID = Constants::standardKeeperInterceptPID();
            break;
        }
    }

    pidMapping[robotID].first.setPID(newPID);
    pidMapping[robotID].second.setPID(newPID);
}
}  // namespace rtt::ai::control