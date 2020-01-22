//
// Created by rolf on 18-01-20.
//

#include "include/roboteam_ai/control/BBTrajectories/BBTrajectory2DAsync.h"
namespace rtt {
template<class num>
BBTrajectory2DAsync<num>::BBTrajectory2DAsync(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, num maxVel, num maxAcc, const LineSegment &line) {
    generateAsyncTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, line);
}
template<class num>
void BBTrajectory2DAsync<num>::generateTrajectory(const Vector2 &initialVel,
        const Vector2 &finalPos, num maxVel, num maxAcc, num alpha) noexcept {
    x = BBTrajectory1D<num>(0, initialVel.x, finalPos.x, maxVel*cosf(alpha), maxAcc*cosf(alpha));
    y = BBTrajectory1D<num>(0, initialVel.y, finalPos.y, maxVel*sinf(alpha), maxAcc*sinf(alpha));
}
template<class num>
void BBTrajectory2DAsync<num>::generateAsyncTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, num maxVel, num maxAcc, const LineSegment &line) noexcept {
    assert(line.length() != 0);// We need to give an input direction
    rotation = line.direction().angle();
    startPosition = initialPos;
    //The idea is to do a binary search over alpha to find a trajectory in x and y direction (which is minimal time)
    num inc = M_PI_4*0.25;
    num alpha = M_PI_4*1.5;
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
template<class num>
Vector2 BBTrajectory2DAsync<num>::getPosition(num t) const {
    return Vector2(x.getPosition(t), y.getPosition(t)).rotate(rotation) + startPosition;
}
template<class num>
std::vector<Vector2> BBTrajectory2DAsync<num>::visCurve() const {
    std::vector<Vector2> points;
    num timeStep = fmax(x.getTotalTime(), y.getTotalTime())/30.0;
    for (int i = 0; i <= 30; ++ i) {
        points.push_back(getPosition(timeStep*i));
    }
    return points;
}
}
