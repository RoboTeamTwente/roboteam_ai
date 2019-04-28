//
// Created by rolf on 17-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include "GetBallCoach.h"
namespace rtt {
namespace ai {
namespace coach {
GetBallCoach getBallCoachObj;
GetBallCoach* getBallCoach = &getBallCoachObj;
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
int GetBallCoach::bestBallGetterID(double secondsAhead) {
    //robot closest to ball that is not keeper
    int closestId = - 1;
    auto closestDist = DBL_MAX;
    auto ball = world::world->getBall();
    Vector2 futureBallPos = ball->pos + ball->vel * secondsAhead;
    for (const auto &robot : world::world->getUs()) {
        if (robot.id != robotDealer::RobotDealer::getKeeperID()) {
            double distToBall = (robot.pos - futureBallPos).length();
            if (distToBall < closestDist) {
                closestDist = distToBall;
                closestId = robot.id;
            }
        }
    }
    return closestId;
}
void GetBallCoach::update() {
    if (shouldWeGetBall()) {
        gettingBall = true;
        idGettingBall = bestBallGetterID(0.0);
    }
    else {
        idGettingBall = - 1;
        gettingBall = false;
    }
}
}
}
}