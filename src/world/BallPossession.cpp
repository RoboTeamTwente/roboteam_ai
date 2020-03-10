//
// Created by rolf on 15-4-19.
//

#include "world/BallPossession.h"
#include <coach/PassCoach.h>
#include <roboteam_utils/Print.h>
#include "world/FieldComputations.h"

namespace rtt::ai {

BallPossession ballPossession;
BallPossession *ballPossessionPtr = &ballPossession;

void BallPossession::update(world_new::view::WorldDataView world, const world::Field &field) {
    updateCloseAndFarTimes(world);
    recomputeState(world, field);
}

void BallPossession::recomputeState(world_new::view::WorldDataView world, const world::Field &field) {
    if (coach::g_pass.getRobotBeingPassedTo() != -1) {
        return;
    }

    bool weAreClose = closeToUsTime > CLOSE_TIME_TRESHOLD;
    bool theyAreClose = closeToThemTime > CLOSE_TIME_TRESHOLD;
    bool weAreFar = farFromUsTime > FAR_TIME_TRESHOLD;
    bool theyAreFar = farFromThemTime > FAR_TIME_TRESHOLD;

    double ourPossessionX = field.getLeftmostX() + OUR_POSSESSION_RELATIVE_LENGTH_THRESHOLD * field.getFieldLength();
    double theirPossessionX = field.getLeftmostX() + THEIR_POSSESSION_RELATIVE_LENGTH_THRESHOLD * field.getFieldLength();
    if ((weAreClose && !theyAreClose) || (world->getBall().value()->getPos().x > ourPossessionX)) {
        state = OURBALL;
    } else if ((theyAreClose && !weAreClose) || (world->getBall().value()->getPos().x < theirPossessionX)) {
        state = THEIRBALL;
    } else if (weAreClose && theyAreClose) {
        state = CONTENDEDBALL;
    } else if (weAreFar && theyAreFar) {
        state = LOOSEBALL;
    }
    // In the other cases we stay in the same state (so we do nothing).
}

void BallPossession::updateCloseAndFarTimes(world_new::view::WorldDataView world) {
    if (!world.getBall()) return;

    double timeDiff = world->getTime() - previousTime;
    previousTime = world->getTime();

    // If a team is close or far to the ball increment the timers, otherwise reset them.
    closeToUsTime = teamCloseToBall(world, true) ? closeToUsTime + timeDiff : 0.0;
    closeToThemTime = teamCloseToBall(world, false) ? closeToThemTime + timeDiff : 0.0;
    farFromUsTime = teamFarFromBall(world, true) ? farFromUsTime + timeDiff : 0.0;
    farFromThemTime = teamFarFromBall(world, false) ? farFromThemTime + timeDiff : 0.0;
}

bool BallPossession::teamCloseToBall(world_new::view::WorldDataView world, bool ourTeam) {
    double closeTreshHoldDist = Constants::MAX_BALL_RANGE();
    auto robots = ourTeam ? world.getUs() : world.getThem();
    for (auto &robot : robots) {
        if (robot.hasBall(closeTreshHoldDist)) {
            return true;
        }
    }
    return false;
}

bool BallPossession::teamFarFromBall(world_new::view::WorldDataView world, bool ourTeam) {
    if (world.getBall()) {
        double farThreshHoldDist = STANDARD_FAR_THRESHOLD;

        if (!ourTeam && world.getBall().value()->getPos().x < this->MIDDLE_LINE_X) {
            // If the ball is on our side, go more defensive.
            farThreshHoldDist = DEFENSIVE_FAR_THRESHOLD;
        }

        auto robots = ourTeam ? world.getUs() : world.getThem();
        for (auto &robot : robots) {
            if ((robot->getPos() - world.getBall().value()->getPos()).length() < farThreshHoldDist) {
                return false;
            }
        }
        return true;
    }
    return false;
}

BallPossession::Possession BallPossession::getPossession() { return state; }

}  // namespace rtt::ai