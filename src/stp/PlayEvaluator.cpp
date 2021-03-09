//
// Created by maxl on 18-02-21.
//

#include "stp/PlayEvaluator.h"
#include <ApplicationManager.h>

#include <stp/invariants/game_states/BallPlacementUsGameStateInvariant.h>
#include <stp/invariants/game_states/BallPlacementThemGameStateInvariant.h>
#include <stp/invariants/game_states/FreeKickThemGameStateInvariant.h>
#include <stp/invariants/game_states/FreeKickUsGameStateInvariant.h>
#include <stp/invariants/game_states/HaltGameStateInvariant.h>
#include <stp/invariants/game_states/KickOffThemGameStateInvariant.h>
#include <stp/invariants/game_states/KickOffThemPrepareGameStateInvariant.h>
#include <stp/invariants/game_states/KickOffUsGameStateInvariant.h>
#include <stp/invariants/game_states/KickOffUsPrepareGameStateInvariant.h>
#include <stp/invariants/game_states/NormalOrFreeKickUsGameStateInvariant.h>
#include <stp/invariants/game_states/NormalPlayGameStateInvariant.h>
#include <stp/invariants/game_states/PenaltyThemGameStateInvariant.h>
#include <stp/invariants/game_states/PenaltyThemPrepareGameStateInvariant.h>
#include <stp/invariants/game_states/PenaltyUsGameStateInvariant.h>
#include <stp/invariants/game_states/PenaltyUsPrepareGameStateInvariant.h>
#include <stp/invariants/game_states/StopGameStateInvariant.h>
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

    uint8_t PlayEvaluator::getGlobalEvaluation(GlobalEvaluation evaluation) {
        return (updatedGlobal.contains(evaluation) ? scoresGlobal.at(evaluation) : scoresGlobal[evaluation] = updateGlobalEvaluation(evaluation));
    }

    uint8_t PlayEvaluator::updateGlobalEvaluation(GlobalEvaluation evaluation) {
        auto _world = world->getWorld().value();

        switch (evaluation) {
            case GlobalEvaluation::BallPlacementThemGameState:
                return invariant::BallPlacementThemGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::BallPlacementUsGameState:
                return invariant::BallPlacementUsGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::FreeKickThemGameState:
                return invariant::FreeKickThemGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::FreeKickUsGameState:
                return invariant::FreeKickUsGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::HaltGameState:
                return invariant::HaltGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffThemGameState:
                return invariant::KickOffThemGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffThemPrepareGameState:
                return invariant::KickOffThemPrepareGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffUsGameState:
                return invariant::KickOffUsGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffUsPrepareGameState:
                return invariant::KickOffUsPrepareGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::NormalOrFreeKickUsGameState:
                return invariant::NormalOrFreeKickUsGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::NormalPlayGameState:
                return invariant::NormalPlayGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyThemGameState:
                return invariant::PenaltyThemGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyThemPrepareGameState:
                return invariant::PenaltyThemPrepareGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyUsGameState:
                return invariant::PenaltyUsGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyUsPrepareGameState:
                return invariant::PenaltyUsPrepareGameStateInvariant().metricCheck(_world, &field);
            case GlobalEvaluation::StopGameState:
                return invariant::StopGameStateInvariant().metricCheck(_world, &field);
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

    void PlayEvaluator::update(world::World* _world) noexcept {
        this->world = _world;
        this->field = _world->getField().value();
    }

    world::World* PlayEvaluator::getWorld() noexcept { return world; }

    void PlayEvaluator::clearGlobalScores() {
        updatedGlobal.clear();
    }

    bool PlayEvaluator::checkInvariant(GlobalEvaluation globalEvaluation, uint8_t cutOff) noexcept { return getGlobalEvaluation(globalEvaluation) >= cutOff; }
}