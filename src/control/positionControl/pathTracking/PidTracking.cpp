//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/PidTracking.h"

namespace rtt::ai::control {

Position PidTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity, std::vector<Vector2> &pathPoints, int robotId) {
    PositionControlUtils::removeFirstIfReached(pathPoints, currentPosition);
    if (pidMapping.find(robotId) == pidMapping.end()){
        pidMapping[robotId] = std::make_pair(PID(), PID());
        pidMapping[robotId].first.setMaxIOutput(MAX_VELOCITY);
        pidMapping[robotId].second.setMaxIOutput(MAX_VELOCITY);
    }
    updatePidValuesFromInterface();

    Vector2 velocity;
    velocity.x = pidMapping[robotId].first.getOutput(currentPosition.x, pathPoints.front().x);
    velocity.y = pidMapping[robotId].second.getOutput(currentPosition.y, pathPoints.front().y);
    return Position(velocity, (pathPoints.front() - currentPosition).angle());
}

void PidTracking::updatePidValuesFromInterface() {
    auto newPid = interface::Output::getNumTreePid();
    for (auto pid: pidMapping){
        pidMapping[pid.first].first.setPID(newPid);
        pidMapping[pid.first].second.setPID(newPid);
    }
}

}  // namespace rtt::ai::control