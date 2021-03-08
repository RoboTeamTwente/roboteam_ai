//
// Created by maxl on 18-02-21.
//

#include "stp/PlayScorer.h"
#include <ApplicationManager.h>

#include <stp/invariants/BallCloseToUsInvariant.h>
#include <stp/invariants/BallCloseToThemInvariant.h>
#include <stp/invariants/BallGotShotInvariant.h>
#include <stp/invariants/BallClosestToUsInvariant.h>
#include <stp/invariants/BallIsFreeInvariant.h>
#include <stp/invariants/BallMovesSlowInvariant.h>
#include <stp/invariants/BallOnOurSideInvariant.h>
#include <stp/invariants/BallOnTheirSideInvariant.h>
#include <stp/invariants/BallShotOrCloseToThemInvariant.h>
#include <stp/invariants/DistanceFromBallInvariant.h>
#include <stp/invariants/GoalVisionFromBallInvariant.h>
#include <stp/invariants/FreedomOfRobotsInvariant.h>
#include <stp/invariants/GoalVisionInvariant.h>
#include <stp/invariants/NoGoalVisionFromBallInvariant.h>
#include <stp/invariants/WeHaveBallInvariant.h>
#include <stp/invariants/WeHaveMajorityInvariant.h>

namespace rtt::ai::stp{

    uint8_t PlayScorer::getGlobalEvaluation(GlobalEvaluation evaluation) {
        int index = static_cast<int>(evaluation);
        return (updatedGlobal[index] ? scoresGlobal[index] : scoresGlobal[index] = updateGlobalEvaluation(index));
    }

    uint8_t PlayScorer::updateGlobalEvaluation(int index) {
        /// A (B at next function)
        auto _field = world->getField();
        auto field = _field.value(); //Overwriting global field for showing the issue
        const auto _world = world->getWorld().value();

        switch ( GlobalEvaluation(index)) {
            case GlobalEvaluation::BallCloseToThem:
                return invariant::BallCloseToThemInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallCloseToUs:
                return invariant::BallCloseToUsInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallClosestToUs:
                return invariant::BallClosestToUsInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallGotShot:
                return invariant::BallGotShotInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallIsFree:
                return invariant::BallIsFreeInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallMovesSlow:
                return invariant::BallMovesSlowInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallOnOurSide:
                return invariant::BallOnOurSideInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallOnTheirSide:
                return invariant::BallOnTheirSideInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallShotOrCloseToThem:
                return invariant::BallShotOrCloseToThemInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::DistanceFromBall:
                return invariant::DistanceFromBallInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::FreedomOfRobots:
                return invariant::FreedomOfRobotsInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::GoalVisionFromBall:
                return invariant::GoalVisionFromBallInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::GoalVision:
                return invariant::GoalVisionInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::NoGoalVisionFromBall:
                return invariant::NoGoalVisionFromBallInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::WeHaveBall:
                return invariant::WeHaveBallInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::WeHaveMajority:
                return invariant::WeHaveMajorityInvariant().metricCheck(_world, &field);
            default:
                RTT_WARNING("Unhandled ScoreEvaluation!");
                return 0;
        }
    }

    void PlayScorer::update(world::World* _world) noexcept {
        this->world = _world;
        /// B
        //this->field = world->getField().value();
    }

    world::World* PlayScorer::getWorld() noexcept { return world; }

    void PlayScorer::clearGlobalScores() {
        for (int i = 0; i < sizeof(GlobalEvaluation); i++) updatedGlobal[i] = false;
    }
}