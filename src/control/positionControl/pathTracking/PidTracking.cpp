//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/PidTracking.h"

namespace rtt::ai::control{

PidTracking::PidTracking(){
    xPid.setMaxIOutput(MAX_VELOCITY);
    yPid.setMaxIOutput(MAX_VELOCITY);
}

Position PidTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                              std::vector<Vector2> &pathPoints) {
    if (pathPoints.size() > 1 && (pathPoints.front() - currentPosition).length() < MIN_DISTANCE){
        pathPoints.erase(pathPoints.begin());
    }
    updatePidValues();

    Vector2 velocity;
    velocity.x = xPid.getOutput(currentPosition.x, pathPoints.front().x);
    velocity.y = yPid.getOutput(currentPosition.y, pathPoints.front().y);
    return Position(velocity, (pathPoints.front() - currentPosition).angle());
}

void PidTracking::updatePidValues(){
    auto newPid = interface::Output::getNumTreePid();
    xPid.setPID(newPid);
    yPid.setPID(newPid);
}

}