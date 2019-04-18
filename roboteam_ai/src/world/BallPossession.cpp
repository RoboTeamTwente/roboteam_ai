//
// Created by rolf on 15-4-19.
//

#include "BallPossession.h"

namespace rtt {
namespace ai {

BallPossession bpTrackerObj;
BallPossession* bpTracker = &bpTrackerObj;

void BallPossession::update() {
    updateTicks();
    recomputeState();
}

void BallPossession::recomputeState() {

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
//    std::cout << stateAsString() << std::endl;
}

void BallPossession::updateTicks() {
    world::WorldData wd = world::world->getWorld();
    double timeDiff = world::world->timeDifference();

    // if a team is close or far to the ball increment the timers, otherwise reset them
    closeToUsTime = teamCloseToBall(wd, true) ? closeToUsTime + timeDiff : 0.0;
    closeToThemTime = teamCloseToBall(wd, false) ? closeToThemTime + timeDiff : 0.0;
    farFromUsTime = teamFarFromBall(wd, true) ? farFromUsTime + timeDiff : 0.0;
    farFromThemTime = teamFarFromBall(wd, false) ? farFromThemTime + timeDiff : 0.0;
}

/// return true if given team is relatively close to ball
bool BallPossession::teamCloseToBall(const world::WorldData &world, bool ourTeam) {
    double closeTreshHoldDist = Constants::MAX_BALL_RANGE();
    std::vector<world::Robot> robots = ourTeam ? world.us : world.them;
    for (auto robot : robots) {
        if (robot.hasBall(closeTreshHoldDist)) {
            return true;
        }
    }
    return false;
}

/// return true if given team is relatively far from ball
bool BallPossession::teamFarFromBall(const world::WorldData &world, bool ourTeam) {
    double farThreshHoldDist = 0.4;
    std::vector<world::Robot> robots = ourTeam ? world.us : world.them;
    for (const auto &robot :robots) {
        if ((robot.pos - world.ball.pos).length() < farThreshHoldDist) {
            return false;
        }
    }
    return true;
}

BallPossession::Possession BallPossession::getPossession() {
    return state;
}

// convert ballpossession states to strings
std::string BallPossession::stateAsString() {
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