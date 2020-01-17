//
// Created by rolf on 17-1-20.
//

#include <gtest/gtest.h>
#include "control/BBTrajectories/BBTrajectory2D.h"

using namespace rtt;
TEST(BBTrajectories, BBTrajectory2D) {

    float inc = M_PI_4*0.5f;
    float alpha = M_PI_4;
    Vector2 startPos(0, 0);
    Vector2 initialVel(- 2, 0);
    Vector2 endPos(4, 4);
    double maxVel=3.0;
    double maxAcc=1.0;
    while (inc>0.000001){
        BBTrajectory2D trajectory(startPos,initialVel,endPos,maxVel,maxAcc,alpha);
        float diff=abs(trajectory.x.getTotalTime()-trajectory.y.getTotalTime());
        if (diff<0.001){
            break;
        }
        if(trajectory.x.getTotalTime()>trajectory.y.getTotalTime()){
            alpha-=inc;
        }
        else{
            alpha+=inc;
        }
        inc*=0.5f;
    }
    BBTrajectory2D endTrajectory(startPos,initialVel,endPos,maxVel,maxAcc,alpha);
    float endTime = endTrajectory.x.getTotalTime();
    for (int i = 0; i <= 30; ++ i) {
        std::cout<<endTrajectory.x.getValues(endTime/30.0*i).pos<<" "<< endTrajectory.y.getValues(endTime/30.0*i).pos<<std::endl;
    }
}