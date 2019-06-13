//
// Created by rolf on 17-4-19.
//

#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>
#include <roboteam_ai/src/world/Field.h>
#include "defence/DefencePositionCoach.h"
#include "GetBallCoach.h"
#include "roboteam_ai/src/interface/api/Input.h"

namespace rtt {
namespace ai {
namespace coach {

GetBallCoach getBallCoachObj;
GetBallCoach* getBallCoach = &getBallCoachObj;

bool GetBallCoach::shouldWeGetBall() {
    // return true if we want to do some ball handling (e.g. harrassing, getting the ball or so). False in other cases
    // should probably listen to ballPossession at some point
    Vector2 ballPos = world::world->getBall()->pos;
    return ! world::field->pointIsInDefenceArea(ballPos, true, 0.04) &&
            ! world::field->pointIsInDefenceArea(ballPos, false) && world::field->pointIsInField(ballPos, - 0.05);
}

bool GetBallCoach::weAreGettingBall() {
    return gettingBall;
}

int GetBallCoach::getBallGetterID() {
    return idGettingBall;
}

int GetBallCoach::bestBallGetterID() {
    auto ball = world::world->getBall();
    double a = 0.5;
    Vector2 ballPos = ball->getBallStillPosition() * (1-a) + ball->pos * a;

    double closestDistSquared = 9e9;
    int closestId = idGettingBall;
    Vector2 closestPos = Vector2();

    auto r = world::world->getRobotForId(idGettingBall);
    if (r) {
        closestPos = r->pos;
        closestDistSquared = (closestPos - ballPos).length2() * 0.65;
    }

    for (const auto &robot : world::world->getUs()) {
        if (robot->id != robotDealer::RobotDealer::getKeeperID() && robot->id != idGettingBall) {
            double distToBallSquared = (robot->pos - ballPos).length2();
            if (distToBallSquared < closestDistSquared) {
                closestDistSquared = distToBallSquared;
                closestId = robot->id;
                closestPos = robot->pos;
            }
        }
    }

    interface::Input::drawData(interface::Visual::BALL_DATA, {closestPos, ballPos}, Qt::lightGray, closestId,
            interface::Drawing::LINES_CONNECTED);

    return closestId;
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

} //coach
} //ai
} //rtt