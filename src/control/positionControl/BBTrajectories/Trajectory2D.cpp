//
// Created by tijmen on 16-12-21.
//

#include "control/positionControl/BBTrajectories/Trajectory2D.h"

#include <cmath>

namespace rtt {

Trajectory2D::Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc, double alpha) {
    BB::BBTrajectory2D BBTNoCollision = BB::BBTrajectory2D(initialPos, initialVel, finalPos, maxVel, maxAcc);
    std::pair<std::vector<BB::BBTrajectoryPart>, std::vector<BB::BBTrajectoryPart>> parts = BBTNoCollision.getParts();
    x.parts = parts.first;
    x.initialPos = initialPos.x;
    x.initialVel = initialVel.x;
    x.finalPos = finalPos.x;
    y.parts = parts.second;
    y.initialPos = initialPos.y;
    y.initialVel = initialVel.y;
    y.finalPos = finalPos.y;
}

void Trajectory2D::addTrajectory(const Trajectory2D &extraTrajectory, double addFromTime) {
    x.addTrajectory(extraTrajectory.x.parts, addFromTime);
    x.finalPos = extraTrajectory.x.finalPos;
    y.addTrajectory(extraTrajectory.y.parts, addFromTime);
    y.finalPos = extraTrajectory.y.finalPos;
}

Vector2 Trajectory2D::getPosition(double t) const { return Vector2(x.getPosition(t), y.getPosition(t)); }

Vector2 Trajectory2D::getVelocity(double t) const { return Vector2(x.getVelocity(t), y.getVelocity(t)); }

Vector2 Trajectory2D::getAcceleration(double t) const { return Vector2(x.getAcceleration(t), y.getAcceleration(t)); }

double Trajectory2D::getTotalTime() const { return std::max(x.getTotalTime(),y.getTotalTime()); }

}