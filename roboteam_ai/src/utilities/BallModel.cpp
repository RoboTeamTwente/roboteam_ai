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
bool BallModel::kickedNow;
int BallModel::ballStraightTicks=0;
void BallModel::updateBallModel(roboteam_msgs::WorldBall newBall) {
    // check for collisions
    Vector2 lastVel=Vector2(currentBall.vel);
    Vector2 newVel= Vector2(newBall.vel);
    //TODO: this model need to be tuned for real life / the noise levels need to be tested esp
    double maxDecelleration=5.0; // m/s^2
    double maxAngleDeviation=5.0/180.0*M_PI;
    double threshold=0.1;// m/s
    double baseNoise=0.2;// m/s
    double slope=0.2 ; // m/s / m/s
    double velNoiseMax;
    if (lastVel.length()<threshold){
        velNoiseMax=baseNoise;
    }
    else {
        velNoiseMax=baseNoise+slope*(lastVel.length()-threshold);
    }

    if ((lastVel-newVel).length()>velNoiseMax){
        // decelleration in the direction; just ball rolling
        double angDif=control::ControlUtils::angleDifference(lastVel.angle(),newVel.angle());
        double minLen=lastVel.length()-maxDecelleration*1/60-velNoiseMax;
        if(angDif<=maxAngleDeviation&&newVel.length()>minLen){
            collidesNow=false;
            ballStraightTicks++;
            kickedNow=false;
        }
        else{
            std::cout<<angDif<< " : "<<maxAngleDeviation<< "|| " << newVel.length() <<" :" <<minLen<<std::endl;
            ballStraightTicks=0;
            // kicked or collided
            if (newVel.length()>(lastVel.length()+velNoiseMax)){
                kickedNow=true;
                std::cout<<"KICKED"<<std::endl;

            }
            else{
                collidesNow=true;
                std::cout<<"COLLIDED"<<std::endl;
            }
        }
    }
    else{
        collidesNow=false;
        ballStraightTicks++;
        kickedNow=false;
    }
    currentBall=newBall;
    std::cout<<ballStraightTicks<<std::endl;
}
bool BallModel::ballCollided() {
    return collidesNow;
}
bool BallModel::isBallInAir() {
    return ballInAir;
}
bool BallModel::ballKicked() {
    return kickedNow;
}

}
}