//
// Created by maxl on 18-02-21.
//

#include "stp/PlayEvaluator.h"
#include <ApplicationManager.h>

#include <stp/evaluations/game_states/BallPlacementUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/BallPlacementThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/FreeKickThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/FreeKickUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/HaltGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffThemPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffUsPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/NormalOrFreeKickUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/NormalPlayGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyThemPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyUsPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/StopGameStateEvaluation.h>
#include <stp/evaluations/global/BallCloseToUsGlobalEvaluation.h>
#include <stp/evaluations/global/BallCloseToThemGlobalEvaluation.h>
#include <stp/evaluations/global/BallGotShotGlobalEvaluation.h>
#include <stp/evaluations/global/BallClosestToUsGlobalEvaluation.h>
#include <stp/evaluations/global/BallIsFreeGlobalEvaluation.h>
#include <stp/evaluations/global/BallMovesSlowGlobalEvaluation.h>
#include <stp/evaluations/global/BallOnOurSideGlobalEvaluation.h>
#include <stp/evaluations/global/BallOnTheirSideGlobalEvaluation.h>
#include <stp/evaluations/global/DistanceFromBallGlobalEvaluation.h>
#include <stp/evaluations/global/GoalVisionFromBallGlobalEvaluation.h>
#include <stp/evaluations/global/FreedomOfRobotsGlobalEvaluation.h>
#include <stp/evaluations/global/GoalVisionGlobalEvaluation.h>
#include <stp/evaluations/global/NoGoalVisionFromBallGlobalEvaluation.h>
#include <stp/evaluations/global/WeHaveBallGlobalEvaluation.h>
#include <stp/evaluations/global/WeHaveMajorityGlobalEvaluation.h>

namespace rtt::ai::stp{

    uint8_t PlayEvaluator::getGlobalEvaluation(GlobalEvaluation evaluation) {
        return (scoresGlobal.contains(evaluation) ? scoresGlobal.at(evaluation) : scoresGlobal[evaluation] = updateGlobalEvaluation(evaluation));
    }

    uint8_t PlayEvaluator::updateGlobalEvaluation(GlobalEvaluation& evaluation) {
        auto _world = world->getWorld().value();
        switch (evaluation) {
            case GlobalEvaluation::BallPlacementThemGameState:
                return evaluation::BallPlacementThemGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallPlacementUsGameState:
                return evaluation::BallPlacementUsGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::FreeKickThemGameState:
                return evaluation::FreeKickThemGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::FreeKickUsGameState:
                return evaluation::FreeKickUsGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::HaltGameState:
                return evaluation::HaltGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffThemGameState:
                return evaluation::KickOffThemGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffThemPrepareGameState:
                return evaluation::KickOffThemPrepareGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffUsGameState:
                return evaluation::KickOffUsGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::KickOffUsPrepareGameState:
                return evaluation::KickOffUsPrepareGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::NormalOrFreeKickUsGameState:
                return evaluation::NormalOrFreeKickUsGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::NormalPlayGameState:
                return evaluation::NormalPlayGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyThemGameState:
                return evaluation::PenaltyThemGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyThemPrepareGameState:
                return evaluation::PenaltyThemPrepareGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyUsGameState:
                return evaluation::PenaltyUsGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::PenaltyUsPrepareGameState:
                return evaluation::PenaltyUsPrepareGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::StopGameState:
                return evaluation::StopGameStateEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallCloseToThem:
                return evaluation::BallCloseToThemGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallCloseToUs:
                return evaluation::BallCloseToUsGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallClosestToUs:
                return evaluation::BallClosestToUsGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallGotShot:
                return evaluation::BallGotShotGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallIsFree:
                return evaluation::BallIsFreeGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallMovesSlow:
                return evaluation::BallMovesSlowGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallOnOurSide:
                return evaluation::BallOnOurSideGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::BallOnTheirSide:
                return evaluation::BallOnTheirSideGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::DistanceFromBall:
                return evaluation::DistanceFromBallGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::FreedomOfRobots:
                return evaluation::FreedomOfRobotsGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::GoalVisionFromBall:
                return evaluation::GoalVisionFromBallGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::GoalVision:
                return evaluation::GoalVisionGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::NoGoalVisionFromBall:
                return evaluation::NoGoalVisionFromBallGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::WeHaveBall:
                return evaluation::WeHaveBallGlobalEvaluation().metricCheck(_world, &field);
            case GlobalEvaluation::WeHaveMajority:
                return evaluation::WeHaveMajorityGlobalEvaluation().metricCheck(_world, &field);
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
        scoresGlobal.clear();
    }

    bool PlayEvaluator::checkEvaluation(GlobalEvaluation globalEvaluation, uint8_t cutOff) noexcept { return getGlobalEvaluation(globalEvaluation) >= cutOff; }

    uint8_t PlayEvaluator::calculateScore(std::vector<PlayScoring>& scoring){
        double scoreTotal,weightTotal = 0;
        for (auto& factor : scoring){
            scoreTotal += factor.evaluationScore;
            weightTotal += factor.weight;
        }
        return scoreTotal/weightTotal;
    }

}