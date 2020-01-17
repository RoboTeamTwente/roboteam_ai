//
// Created by rolf on 17-1-20.
//

#include "include/roboteam_ai/control/BBTrajectories/BBTrajectory2D.h"
namespace rtt {
BBTrajectory2D::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
        float maxVel, float maxAcc, float alpha) {
    generateTrajectory(initialPos, initialVel, finalPos, maxVel, maxAcc, alpha);
}
BBTrajectory2D::BBTrajectory2D(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
        float maxVel, float maxAcc) {
    generateSyncedTrajectory(initialPos,initialVel,finalPos,maxVel,maxAcc);
}
void BBTrajectory2D::generateTrajectory(const Vector2 &initialPos, const Vector2 &initialVel, const Vector2 &finalPos,
        float maxVel,
        float maxAcc, float alpha) {
    x.generateTrajectory(initialPos.x, initialVel.x, finalPos.x, maxVel*cosf(alpha), maxAcc*cosf(alpha));
    y.generateTrajectory(initialPos.x, initialVel.x, finalPos.x, maxVel*sinf(alpha), maxAcc*sinf(alpha));
}
void BBTrajectory2D::generateSyncedTrajectory(const Vector2 &initialPos, const Vector2 &initialVel,
        const Vector2 &finalPos, float maxVel, float maxAcc) {
    //The idea is to do a binary search over alpha to find a trajectory in x and y direction (which is minimal time)
    float inc = M_PI_4*0.5f;
    float alpha = M_PI_4;
    //TODO: tune magic numbers here.
    while(inc>1e-7){
        generateTrajectory(initialPos,initialVel,finalPos,maxVel,maxAcc,alpha);
        float diff=abs(x.getTotalTime()-y.getTotalTime());
        //If the trajectories match enough we stop earlier
        if (diff<0.001){
            return;
        }
        if (x.getTotalTime()>y.getTotalTime()){
            alpha-=inc;
        } else{
            alpha+=inc;
        }
        inc*=0.5f;
    }
}

}