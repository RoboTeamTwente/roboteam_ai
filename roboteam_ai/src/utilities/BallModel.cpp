//
// Created by rolf on 13-3-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "BallModel.h"
namespace rtt{
namespace ai{
/// what do we want:
/// Inelastic collision detection. Kick detection. Chip detection and prediction (maybe even height estimation).
/// Time prediction (deceleration)
roboteam_msgs::WorldBall BallModel::currentBall;
bool BallModel::ballInAir;
double BallModel::lastKickVel;
bool BallModel::collidesNow;
void BallModel::updateBallModel(roboteam_msgs::WorldBall newBall) {
    double maxDecelleration=5.0; // m/s^2
    double lastVel=Vector2(currentBall.vel).length();
    double velNoiseMax;
    if (lastVel<0.1){
        velNoiseMax=0.02;
    }
    else {
        velNoiseMax=0.3*lastVel;
    }
    double maxAngleDeviation=5.0/180.0*M_PI;
    double newVel= Vector2(newBall.vel).length();
    if (control::ControlUtils::angleDifference(Vector2(currentBall.vel).angle(),Vector2(newBall.vel).angle())>=maxAngleDeviation){
        collidesNow=true;
    }
    else if(newVel>(velNoiseMax+lastVel)||newVel<(lastVel-maxDecelleration*1/60-velNoiseMax)){
        collidesNow=true;
    }
    else{
        collidesNow=false;
    }


}

}
}