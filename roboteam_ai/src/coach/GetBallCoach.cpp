//
// Created by rolf on 17-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "GetBallCoach.h"
namespace rtt{
namespace ai{
namespace coach{
GetBallCoach GBCoachObj;
GetBallCoach* GBCoach=&GBCoachObj;
bool GetBallCoach::shouldWeGetBall() {
    // return true if we want to do some ball handling (e.g. harrassing, getting the ball or so). False in other cases
    // should probably listen to ballPossession at some point
    return true;
}
bool GetBallCoach::weAreGettingBall() {
    return gettingBall;
}
int GetBallCoach::getBallGetterID() {
    return idGettingBall;
}
int GetBallCoach::bestBallGetterID() {
   //robot closest to ball that is not keeper
   int closestId=-1;
   double closestDist=DBL_MAX;
   Vector2 ballPos=world::world->getBall()->pos;
   for (const auto& robot : world::world->getUs()){
       if (robot.id!=robotDealer::RobotDealer::getKeeperID()){
           double distToBall=(robot.pos-ballPos).length();
           if (distToBall<closestDist){
               closestDist=distToBall;
               closestId=robot.id;
           }
       }
   }
   return closestId;
}
void GetBallCoach::update(){
    if (shouldWeGetBall()){
        gettingBall=true;
        idGettingBall=bestBallGetterID();
    }
    else{
        idGettingBall=-1;
        gettingBall=false;
    }
}
}
}
}