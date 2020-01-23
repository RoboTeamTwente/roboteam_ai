//
// Created by ratoone on 12-11-19.
//

#include <control/positionControl/pathTracking/BasicPathTracking.h>

void BasicPathTracking::trackPath(const rtt::Vector2 &currentPosition, const rtt::Vector2 &currentVelocity,
        std::list<rtt::Vector2> &pathPoints, rtt::Vector2 &outputVelocity, double &outputAngle) {
    if (pathPoints.size() > 1 && (pathPoints.front() - currentPosition).length() < minimumDistance){
        pathPoints.pop_front();
    }

    rtt::Vector2 velocity = pathPoints.front() - currentPosition;

    if (velocity.length() > maxVelocity){
        velocity = velocity.stretchToLength(maxVelocity);
    }

    outputVelocity = velocity;
    outputAngle = velocity.angle();
}