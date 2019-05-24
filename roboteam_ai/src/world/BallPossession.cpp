//
// Created by rolf on 15-4-19.
//

#include "BallPossession.h"
#include <roboteam_ai/src/coach/PassCoach.h>

namespace rtt {
namespace ai {

BallPossession ballPossession;
BallPossession* ballPossessionPtr = &ballPossession;

void BallPossession::update() {
    updateTicks();
    recomputeState();
}

void BallPossession::recomputeState() {

    if (coach::g_pass.getRobotBeingPassedTo() != -1) {
        return;
    }

    bool weAreClose = closeToUsTime > CLOSE_TIME_TRESHOLD;
    bool theyAreClose = closeToThemTime > CLOSE_TIME_TRESHOLD;
    bool weAreFar = farFromUsTime > FAR_TIME_TRESHOLD;
    bool theyAreFar = farFromThemTime > FAR_TIME_TRESHOLD;

    if (weAreClose && ! theyAreClose) {
        state = OURBALL;
    }
    else if (theyAreClose && ! weAreClose) {
        state = THEIRBALL;
    }
    else if (weAreClose && theyAreClose) {
        state = CONTENDEDBALL;
    }
    else if (weAreFar && theyAreFar) {
        state = LOOSEBALL;
    }
    // in the other cases we stay in the same state (so we do nothing)
}

void BallPossession::updateTicks() {
    auto wd = world::world->getWorld();
    if (!wd.ball) return;

    double timeDiff = world::world->getTimeDifference();

    // if a team is close or far to the ball increment the timers, otherwise reset them
    closeToUsTime = teamCloseToBall(wd, true) ? closeToUsTime + timeDiff : 0.0;
    closeToThemTime = teamCloseToBall(wd, false) ? closeToThemTime + timeDiff : 0.0;
    farFromUsTime = teamFarFromBall(wd, true) ? farFromUsTime + timeDiff : 0.0;
    farFromThemTime = teamFarFromBall(wd, false) ? farFromThemTime + timeDiff : 0.0;
}

/// return true if given team is relatively close to ball
bool BallPossession::teamCloseToBall(const world::WorldData &world, bool ourTeam) {
    double closeTreshHoldDist = Constants::MAX_BALL_RANGE();
    auto robots = ourTeam ? world.us : world.them;
    for (auto &robot : robots) {
        if (robot->hasBall(closeTreshHoldDist)) {
            return true;
        }
    }
    return false;
}

/// return true if given team is relatively far from ball
bool BallPossession::teamFarFromBall(const world::WorldData &world, bool ourTeam) {
    double farThreshHoldDist = 0.4;
    auto robots = ourTeam ? world.us : world.them;
    for (auto &robot : robots) {
        if ((robot->pos - world.ball->pos).length() < farThreshHoldDist) {
            return false;
        }
    }
    return true;
}

BallPossession::Possession BallPossession::getPossession() {
    return state;
}

// convert ballpossession states to strings
std::string BallPossession::stateAsString(Possession state) {
    switch (state) {
    case OURBALL:return "OURBALL";
    case THEIRBALL:return "THEIRBALL";
    case CONTENDEDBALL:return "CONTENDEDBALL";
    case LOOSEBALL:return "LOOSE";
    default:return "LOOSE";
    }
}

} // ai
} // rtt