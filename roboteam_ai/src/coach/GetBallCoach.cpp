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
    // listen to world_state e.g. if ball is loose we try to get it with one of our robots
    return true;
}
bool GetBallCoach::areWeGettingBall() {
    return gettingBall;
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