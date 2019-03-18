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
bool BallModel::ballInAir=false;
double BallModel::lastKickVel;
bool BallModel::collidesNow=false;
bool BallModel::kickedNow=false;
bool BallModel::dribbledNow=false;
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
    updateDribbling(newBall);
    currentBall = newBall;

}
void BallModel::updateDribbling(roboteam_msgs::WorldBall newBall) {
    // first find the robot that is most likely dribbling the ball. This can possibly be moved to world
    std::pair<int,bool> bestBot=std::make_pair(-1,true);

    //TODO: tune constants and move to Constants
    double maxDribbleRange=0.05;
    double maxSpeedDiff=0.5;
    double bestRange=maxDribbleRange;
    for (auto bot :World::get_world().us){
        if (World::OurBotsBall.find(bot.id)!=World::OurBotsBall.end()){
            if (World::OurBotsBall[bot.id]<=maxDribbleRange&&World::OurBotsBall[bot.id]<=bestRange){
                bestRange=World::OurBotsBall[bot.id];
                bestBot=std::make_pair(bot.id,true);
            }
        }
    }
    for (auto bot :World::get_world().them){
        if (World::TheirBotsBall.find(bot.id)!=World::TheirBotsBall.end()){
            if (World::TheirBotsBall[bot.id]<=maxDribbleRange&&World::TheirBotsBall[bot.id]<=bestRange){
                bestRange=World::TheirBotsBall[bot.id];
                bestBot=std::make_pair(bot.id,true);
            }
        }
    }
    if (bestBot.first==-1){
        dribbledNow=false;
        return;
    }
    // Model that compares robot and ball velocities and estimates whether or not the ball is being dribbled
    std::shared_ptr<roboteam_msgs::WorldRobot> likelyDribblingBot=World::getRobotForId(bestBot.first,bestBot.second);
    if (!likelyDribblingBot){
        dribbledNow=false;
        ROS_ERROR("Could not find the dribbling robot in the worldState!" );
        return;
    }
    Vector2 ballTouchPointVel=likelyDribblingBot->vel;
    ballTouchPointVel=ballTouchPointVel+Vector2((Constants::BALL_RADIUS()+Constants::ROBOT_RADIUS())*2*M_PI*likelyDribblingBot->w,0).rotate(likelyDribblingBot->angle+M_PI_2);// takes into account tha the point on the robot dribbles. can be commented out for now but useful against e.g. zjunlict who rotate with ball
    dribbledNow= (Vector2(newBall.vel)-ballTouchPointVel).length() <= maxSpeedDiff;
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
bool BallModel::ballDribbled(){
    return dribbledNow;
}


}
}