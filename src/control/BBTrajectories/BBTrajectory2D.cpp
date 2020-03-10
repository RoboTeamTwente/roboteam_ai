//
// Created by rolf on 17-1-20.
//

#include "include/roboteam_ai/control/BBTrajectories/BBTrajectory2D.h"
#include <cmath>

namespace rtt {

BBTrajectory2D::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
        double maxVel, double maxAcc, double alpha)  {
    generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, alpha);
}

BBTrajectory2D::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
        double maxVel, double maxAcc)  {
    generateSyncedTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc);
}
 void BBTrajectory2D::generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos,
        double maxVel, double maxAcc, double alpha)  {
    x = BBTrajectory1D(initialPos.x, initialVel.x, finalPos.x, maxVel*cosf(alpha), maxAcc*cosf(alpha));
    y = BBTrajectory1D(initialPos.y, initialVel.y, finalPos.y, maxVel*sinf(alpha), maxAcc*sinf(alpha));
}

void BBTrajectory2D::generateSyncedTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, double maxVel, double maxAcc)  {
    //The idea is to do a binary search over alpha to find a trajectory in x and y direction (which is minimal time)
    float inc = M_PI_4*0.5;
    float alpha = M_PI_4;
    //TODO: tune convergence numbers
    while (inc > 1e-7) {
        generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, alpha);
        float diff = abs(x.getTotalTime() - y.getTotalTime());
        //If the trajectories match enough we stop earlier
        if (diff < 0.001) {
            return;
        }
        if (x.getTotalTime() > y.getTotalTime()) {
            alpha -= inc;
        }
        else {
            alpha += inc;
        }
        inc *= 0.5;
    }
}

Vector2 BBTrajectory2D::getPosition(double t) const {
    return Vector2(x.getPosition(t),y.getPosition(t));
}

Vector2 BBTrajectory2D::getVelocity(double t) const {
    return Vector2(x.getVelocity(t),y.getVelocity(t));
}

Vector2 BBTrajectory2D::getAcceleration(double t) const {
    return Vector2(x.getAcceleration(t),y.getAcceleration(t));
}

std::vector<Vector2> BBTrajectory2D::visCurve() const {
    std::vector<Vector2> points;
    double timeStep=fmax(x.getTotalTime(),y.getTotalTime())/30.0;
    for (int i = 0; i <= 30; ++ i) {
        points.push_back(getPosition(timeStep*i));
    }
    return points;
}
}