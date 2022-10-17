//
// Created by tijmen on 16-12-21.
//

#include "control/positionControl/BBTrajectories/Trajectory2D.h"

#include <cmath>

#include "control/positionControl/BBTrajectories/BBTrajectory2D.h"
#include "roboteam_utils/Print.h"

namespace rtt {

Trajectory2D::Trajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos, double maxVel, double maxAcc) {
    BB::BBTrajectory2D BBTNoCollision = BB::BBTrajectory2D(initialPos, initialVel, finalPos, maxVel, maxAcc);
    std::pair<std::vector<BB::BBTrajectoryPart>, std::vector<BB::BBTrajectoryPart>> parts = BBTNoCollision.getParts();
    x.parts = parts.first;
    x.finalPos = finalPos.x;
    y.parts = parts.second;
    y.finalPos = finalPos.y;
}

void Trajectory2D::addTrajectory(const Trajectory2D &extraTrajectory, double addFromTime) {
    x.addTrajectory(extraTrajectory.x.parts, addFromTime);
    x.finalPos = extraTrajectory.x.finalPos;
    y.addTrajectory(extraTrajectory.y.parts, addFromTime);
    y.finalPos = extraTrajectory.y.finalPos;
}

std::vector<Vector2> Trajectory2D::getPathApproach(double timeStep) const {
    std::vector<Vector2> points;
    auto totalTime = getTotalTime();
    double time = 0;

    if (totalTime == std::numeric_limits<double>::infinity()) {
        // TODO: Prevent this from happening!
        RTT_ERROR("Infinite while loop")
        // throw std::runtime_error("Total time of infinity!");
        return {getPosition(0)};
    }

    while (time <= totalTime) {
        time += timeStep;
        points.push_back(getPosition(time));
    }
    return points;
}

std::vector<Vector2> Trajectory2D::getVelocityVector(double timeStep) const {
    std::vector<Vector2> velocities;
    auto totalTime = getTotalTime();
    double time = 0;

    if (totalTime == std::numeric_limits<double>::infinity()) {
        // TODO: Prevent this from happening!
        RTT_ERROR("Infinite while loop")
        // throw std::runtime_error("Total time of infinity!");
        return {getPosition(0)};
    }

    while (time <= totalTime) {
        time += timeStep;
        velocities.push_back(getVelocity(time));
    }
    return velocities;
}

Vector2 Trajectory2D::getPosition(double t) const { return Vector2(x.getPosition(t), y.getPosition(t)); }

Vector2 Trajectory2D::getVelocity(double t) const { return Vector2(x.getVelocity(t), y.getVelocity(t)); }

Vector2 Trajectory2D::getAcceleration(double t) const { return Vector2(x.getAcceleration(t), y.getAcceleration(t)); }

double Trajectory2D::getTotalTime() const { return std::max(x.getTotalTime(), y.getTotalTime()); }

}  // namespace rtt