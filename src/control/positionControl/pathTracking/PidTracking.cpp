//
// Created by ratoone on 24-01-20.
//

#include "control/positionControl/pathTracking/PidTracking.h"

namespace rtt::ai::control{

PidTracking::PidTracking(){
    xPid.setMaxIOutput(maxVelocity);
    yPid.setMaxIOutput(maxVelocity);
}

void PidTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity,
                                              std::list<Vector2> &pathPoints, Vector2 &outputVelocity,
                                              double &outputAngle) {
    if (pathPoints.size() > 1 && (pathPoints.front() - currentPosition).length() < minimumDistance){
        pathPoints.pop_front();
    }
    updatePidValues();

    outputVelocity.x = xPid.getOutput(currentPosition.x, pathPoints.front().x);
    outputVelocity.y = yPid.getOutput(currentPosition.y, pathPoints.front().y);
    outputAngle = outputVelocity.angle();
}

void PidTracking::updatePidValues(){
    auto newPid = interface::Output::getNumTreePid();
    xPid.setPID(newPid);
    yPid.setPID(newPid);
}

}