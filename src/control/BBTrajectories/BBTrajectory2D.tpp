//
// Created by rolf on 17-1-20.
//

#include "include/roboteam_ai/control/BBTrajectories/BBTrajectory2D.h"
namespace rtt {
template<class num>
BBTrajectory2D<num>::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
        num maxVel, num maxAcc, num alpha) noexcept {
    generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, alpha);
}
template<class num>
BBTrajectory2D<num>::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
        num maxVel, num maxAcc) noexcept {
    generateSyncedTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc);
}
template<class num> void BBTrajectory2D<num>::generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos,
        num maxVel, num maxAcc, num alpha) noexcept {
    x = BBTrajectory1D<num>(initialPos.x, initialVel.x, finalPos.x, maxVel*cosf(alpha), maxAcc*cosf(alpha));
    y = BBTrajectory1D<num>(initialPos.y, initialVel.y, finalPos.y, maxVel*sinf(alpha), maxAcc*sinf(alpha));
}
template<class num>
void BBTrajectory2D<num>::generateSyncedTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, num maxVel, num maxAcc) noexcept {
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

}