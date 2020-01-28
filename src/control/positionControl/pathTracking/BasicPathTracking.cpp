//
// Created by ratoone on 12-11-19.
//

#include "control/positionControl/pathTracking/BasicPathTracking.h"

namespace rtt::ai::control{

void BasicPathTracking::trackPath(const Vector2 &currentPosition, const Vector2 &currentVelocity,
        std::vector<Vector2> &pathPoints, Vector2 &outputVelocity, double &outputAngle) {
    if (pathPoints.size() > 1 && (pathPoints.front() - currentPosition).length() < minimumDistance){
        pathPoints.erase(pathPoints.begin());
    }

    Vector2 velocity = pathPoints.front() - currentPosition;

    if (velocity.length() > maxVelocity){
        velocity = velocity.stretchToLength(maxVelocity);
    }

    outputVelocity = velocity;
    outputAngle = velocity.angle();
}
}