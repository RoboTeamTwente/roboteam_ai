//
// Created by rolf on 17-4-19.
//

#include "coach/GetBallCoach.h"
#include <utilities/RobotDealer.h>
#include <world/FieldComputations.h>
#include <world/World.h>
#include "coach/defence/DefencePositionCoach.h"
#include "interface/api/Input.h"

namespace rtt::ai::coach {

GetBallCoach getBallCoachObj;
GetBallCoach *getBallCoach = &getBallCoachObj;

bool GetBallCoach::shouldWeGetBall(const Field &field) {
    // return true if we want to do some ball handling (e.g. harrassing, getting the ball or so). False in other cases
    // should probably listen to ballPossession at some point
    Vector2 ballPos = world::world->getBall()->getPos();
    return !FieldComputations::pointIsInDefenceArea(field, ballPos, true, 0.04) && !FieldComputations::pointIsInDefenceArea(field, ballPos, false) &&
           FieldComputations::pointIsInField(field, ballPos, -0.05);
}

bool GetBallCoach::weAreGettingBall() { return gettingBall; }

int GetBallCoach::getBallGetterID() { return idGettingBall; }

int GetBallCoach::bestBallGetterID() {
    auto ball = world::world->getBall();
    double a = 0.5;
    Vector2 ballPos = ball->getExpectedBallEndPosition() * (1 - a) + ball->getPos() * a;

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

    interface::Input::drawData(interface::Visual::BALL_DATA, {closestPos, ballPos}, Qt::lightGray, closestId, interface::Drawing::LINES_CONNECTED);

    return closestId;
}

void GetBallCoach::update(const Field &field) {
    if (shouldWeGetBall(field)) {
        gettingBall = true;
        idGettingBall = bestBallGetterID();
    } else {
        idGettingBall = -1;
        gettingBall = false;
    }
}

}  // namespace rtt::ai::coach