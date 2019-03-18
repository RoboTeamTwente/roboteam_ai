//
// Created by rolf on 13-3-19.
//

#include <roboteam_ai/src/control/ControlUtils.h>
#include "BallModel.h"
namespace rtt {
namespace ai {
/// what do we want:
/// Inelastic collision detection. Kick detection. Chip detection and prediction (maybe even height estimation).
/// Time prediction (deceleration)
roboteam_msgs::WorldBall BallModel::currentBall;
bool BallModel::ballInAir;
double BallModel::lastKickVel;
bool BallModel::collidesNow;
bool BallModel::kickedNow;
int BallModel::ballStraightTicks = 0;
void BallModel::updateBallModel(roboteam_msgs::WorldBall newBall) {
    // check for collisions
    Vector2 lastVel = Vector2(currentBall.vel);
    Vector2 newVel = Vector2(newBall.vel);
    //TODO: this model need to be tuned for real life / the noise levels need to be tested esp
    double maxDecelleration = 5.0; // m/s^2
    double linearDeviation, orthogonalDeviation;
    //noise rectangle; rectangle within the velocity state diagram where the velocity is still considered 'good'
    double linearTreshhold=0.1;
    double baseLinear=0.1;
    double linearSlope=0.1;
    double orthogonaltreshHold=0.1;
    double baseOrthogo=0.1;
    double orthogoSlope=0.01;
    double linearLen=0;
    if (lastVel.length()!=0&&newVel.length()!=0) {
        Vector2 projection = newVel.project2(lastVel);
        linearLen = projection.length();
    }
    double hypotLen=newVel.length();
    double orthogoLen=0;
    if (hypotLen>=linearLen) {
        orthogoLen = sqrt(hypotLen*hypotLen - linearLen*linearLen);
    }

    if (linearLen<linearTreshhold){
        linearDeviation=baseLinear;
    }
    else{
        linearDeviation=baseLinear+linearSlope*(linearLen-linearTreshhold);
    }
    if (orthogoLen<orthogonaltreshHold){
        orthogonalDeviation=baseOrthogo;
    }
    else{
        orthogonalDeviation=baseOrthogo+orthogoSlope*(orthogoLen-orthogonaltreshHold);
    }

    // defining the rectangle
    Vector2 centerLow = lastVel - lastVel.stretchToLength(linearDeviation + maxDecelleration/60);
    Vector2 centerHigh = lastVel + lastVel.stretchToLength(linearDeviation);
    Vector2 orthogoVec = Vector2(orthogonalDeviation, 0).rotate(lastVel.angle() + M_PI_2);
    Vector2 point1 = centerLow + orthogoVec;
    Vector2 point2 = centerLow - orthogoVec;
    Vector2 point3 = centerHigh - orthogoVec;
    Vector2 point4 = centerHigh + orthogoVec;

    if (!control::ControlUtils::pointInRectangle(newVel,point1,point2,point3,point4)) {
        // decelleration in the direction; just ball rolling

        ballStraightTicks = 0;
        // kicked or collided
        if (newVel.length() > (lastVel.length()+linearDeviation)) {
            kickedNow = true;
            std::cout << "KICKED" << std::endl;
        }
        else {
            collidesNow = true;
            std::cout << "COLLIDED" << std::endl;
        }
    }
    else {
        collidesNow = false;
        ballStraightTicks ++;
        kickedNow = false;
    }
    currentBall = newBall;
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