//
// Created by rolf on 15-4-19.
//

#include "world/BallPossession.h"
#include "world/WorldData.h"
#include "world/FieldComputations.h"
#include <coach/PassCoach.h>

namespace rtt::ai {

    BallPossession ballPossession;
    BallPossession *ballPossessionPtr = &ballPossession;

    void BallPossession::update(const Field &field) {
        updateCloseAndFarTimes();
        recomputeState(field);
    }

    void BallPossession::recomputeState(const Field &field) {
        if (coach::g_pass.getRobotBeingPassedTo() != -1) {
            return;
        }

        bool weAreClose = closeToUsTime > CLOSE_TIME_TRESHOLD;
        bool theyAreClose = closeToThemTime > CLOSE_TIME_TRESHOLD;
        bool weAreFar = farFromUsTime > FAR_TIME_TRESHOLD;
        bool theyAreFar = farFromThemTime > FAR_TIME_TRESHOLD;

        double ourPossessionX = field.getLeftmostX() + OUR_POSSESSION_RELATIVE_LENGTH_THRESHOLD * field.getFieldLength();
        double theirPossessionX = field.getLeftmostX() + THEIR_POSSESSION_RELATIVE_LENGTH_THRESHOLD * field.getFieldLength();
        if ((weAreClose && !theyAreClose) || (world::world->getBall()->getPos().x > ourPossessionX)) {
            state = OURBALL;
        } else if ((theyAreClose && !weAreClose) || (world::world->getBall()->getPos().x < theirPossessionX)) {
            state = THEIRBALL;
        } else if (weAreClose && theyAreClose) {
            state = CONTENDEDBALL;
        } else if (weAreFar && theyAreFar) {
            state = LOOSEBALL;
        }
        // In the other cases we stay in the same state (so we do nothing).
    }

    void BallPossession::updateCloseAndFarTimes() {
        auto wd = world::world->getWorld();
        if (!wd.ball) return;

        double timeDiff = world::world->getTimeDifference();

        // If a team is close or far to the ball increment the timers, otherwise reset them.
        closeToUsTime = teamCloseToBall(wd, true) ? closeToUsTime + timeDiff : 0.0;
        closeToThemTime = teamCloseToBall(wd, false) ? closeToThemTime + timeDiff : 0.0;
        farFromUsTime = teamFarFromBall(wd, true) ? farFromUsTime + timeDiff : 0.0;
        farFromThemTime = teamFarFromBall(wd, false) ? farFromThemTime + timeDiff : 0.0;
    }

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

    bool BallPossession::teamFarFromBall(const world::WorldData &world, bool ourTeam) {
        if (world.ball) {
            double farThreshHoldDist = STANDARD_FAR_THRESHOLD;

            if (!ourTeam && world.ball->getPos().x < this->MIDDLE_LINE_X) {
                // If the ball is on our side, go more defensive.
                farThreshHoldDist = DEFENSIVE_FAR_THRESHOLD;
            }

            auto robots = ourTeam ? world.us : world.them;
            for (auto &robot : robots) {
                if ((robot->pos - world.ball->getPos()).length() < farThreshHoldDist) {
                    return false;
                }
            }
            return true;
        }
        return false;
    }

    BallPossession::Possession BallPossession::getPossession() {
        return state;
    }

}  // namespace rtt::ai