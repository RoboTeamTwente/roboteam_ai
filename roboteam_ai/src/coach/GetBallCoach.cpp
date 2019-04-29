//
// Created by rolf on 17-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/src/world/Field.h>
#include "defence/DefencePositionCoach.h"
#include "GetBallCoach.h"

namespace rtt {
namespace ai {
namespace coach {
GetBallCoach getBallCoachObj;
GetBallCoach* getBallCoach = &getBallCoachObj;
bool GetBallCoach::shouldWeGetBall() {
    // return true if we want to do some ball handling (e.g. harrassing, getting the ball or so). False in other cases
    // should probably listen to ballPossession at some point
    Vector2 ballPos= world::world->getBall()->pos;
    return ! (world::field->pointIsInDefenceArea(ballPos, true, 0.04)
            || world::field->pointIsInDefenceArea(ballPos, false));
}
bool GetBallCoach::weAreGettingBall() {
    return gettingBall;
}
int GetBallCoach::getBallGetterID() {
    return idGettingBall;
}
int GetBallCoach::bestBallGetterID() {
    //robot closest to ball that is not keeper
    int closestId = - 1;
    double closestDist = DBL_MAX;
    Vector2 ballPos = world::world->getBall()->pos;
    for (const auto &robot : world::world->getUs()) {
        if (!(robot.id == robotDealer::RobotDealer::getKeeperID()||checkDefender(robot.id))){
            double distToBall = (robot.pos - ballPos).length();
            if (distToBall < closestDist) {
                closestDist = distToBall;
                closestId = robot.id;
            }
        }
    }
    return closestId;
}
bool GetBallCoach::checkDefender(int id) {
    if (world::world->getBall()->pos.x>g_defensivePositionCoach.maxX()){
        auto set=robotDealer::RobotDealer::findRobotsForTactic("TestD");
        if (set.find(id)!=set.end()){
            return true;
        }
    }
    return false;
}
void GetBallCoach::update() {
    if (shouldWeGetBall()) {
        gettingBall = true;
        idGettingBall = bestBallGetterID();
    }
    else {
        idGettingBall = - 1;
        gettingBall = false;
    }
}
}
}
}