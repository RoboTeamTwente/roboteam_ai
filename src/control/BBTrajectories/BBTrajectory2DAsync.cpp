//
// Created by rolf on 18-01-20.
//

#include "include/roboteam_ai/control/BBTrajectories/BBTrajectory2DAsync.h"
#include <cmath>

namespace rtt {

BBTrajectory2DAsync::BBTrajectory2DAsync(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, double maxVel, double maxAcc, const LineSegment &line) {
    generateAsyncTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, line);
}

void BBTrajectory2DAsync::generateTrajectory(const Vector2 &initialVel,
        const Vector2 &finalPos, double maxVel, double maxAcc, double alpha) noexcept {
    x = BBTrajectory1D(0, initialVel.x, finalPos.x, maxVel*cosf(alpha), maxAcc*cosf(alpha));
    y = BBTrajectory1D(0, initialVel.y, finalPos.y, maxVel*sinf(alpha), maxAcc*sinf(alpha));
}

void BBTrajectory2DAsync::generateAsyncTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, double maxVel, double maxAcc, const LineSegment &line) noexcept {
    assert(line.length() != 0);// We need to give an input direction
    rotation = line.direction().angle();
    startPosition = initialPos;
    //The idea is to do a binary search over alpha to find a trajectory in x and y direction (which is minimal time)
    double inc = M_PI_4*0.25;
    double alpha = M_PI_4*1.5;
    Vector2 lineStart = (line.start - initialPos).rotate(- rotation);
    Vector2 lineEnd = (line.end - initialPos).rotate(- rotation);
    LineSegment checkLine(Vector2(lineStart.x,0),Vector2(lineEnd.x,0));
    Vector2 endPos = (finalPos - initialPos).rotate(- rotation);
    Vector2 rotatedInitialVel = initialVel.rotate(- rotation);
    //the endtime for y monotonically decreases with alpha so the goal is to find the highest alpha such that the constraints are fulfilled
    while (inc > 1e-5) {
        generateTrajectory(rotatedInitialVel,endPos, maxVel, maxAcc,alpha);
        //If the trajectories match enough we stop earlier
        float diff = abs(x.getTotalTime() - y.getTotalTime());
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

Vector2 BBTrajectory2DAsync::getPosition(double t) const {
    return Vector2(x.getPosition(t), y.getPosition(t)).rotate(rotation) + startPosition;
}

std::vector<Vector2> BBTrajectory2DAsync::visCurve() const {
    std::vector<Vector2> points;
    double timeStep = fmax(x.getTotalTime(), y.getTotalTime())/30.0;
    for (int i = 0; i <= 30; ++ i) {
        points.push_back(getPosition(timeStep*i));
    }
    return points;
}
}
