//
// Created by maxl on 18-02-21.
//

#include "stp/PlayEvaluator.h"

#include <STPManager.h>
#include <stp/evaluations/game_states/BallPlacementThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/BallPlacementUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/FreeKickThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/FreeKickUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/HaltGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffThemPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffUsOrNormalGameStateEvaluation.h>
#include <stp/evaluations/game_states/KickOffUsPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/NormalOrFreeKickUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/NormalPlayGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyThemGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyThemPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyUsGameStateEvaluation.h>
#include <stp/evaluations/game_states/PenaltyUsPrepareGameStateEvaluation.h>
#include <stp/evaluations/game_states/StopGameStateEvaluation.h>
#include <stp/evaluations/game_states/TimeOutGameStateEvaluation.h>
#include <stp/evaluations/global/BallCloseToThemGlobalEvaluation.h>
#include <stp/evaluations/global/BallCloseToUsGlobalEvaluation.h>
#include <stp/evaluations/global/BallClosestToUsGlobalEvaluation.h>
#include <stp/evaluations/global/BallGotShotGlobalEvaluation.h>
#include <stp/evaluations/global/BallInOurDefenseAreaAndStillGlobalEvaluation.h>
#include <stp/evaluations/global/BallIsFreeGlobalEvaluation.h>
#include <stp/evaluations/global/BallMovesSlowGlobalEvaluation.h>
#include <stp/evaluations/global/BallNotInOurDefenseAreaAndStillGlobalEvaluation.h>
#include <stp/evaluations/global/BallOnOurSideGlobalEvaluation.h>
#include <stp/evaluations/global/BallOnTheirSideGlobalEvaluation.h>
#include <stp/evaluations/global/DistanceFromBallGlobalEvaluation.h>
#include <stp/evaluations/global/FreedomOfRobotsGlobalEvaluation.h>
#include <stp/evaluations/global/GoalVisionFromBallGlobalEvaluation.h>
#include <stp/evaluations/global/GoalVisionGlobalEvaluation.h>
#include <stp/evaluations/global/NoGoalVisionFromBallGlobalEvaluation.h>
#include <stp/evaluations/global/TheyDoNotHaveBallGlobalEvaluation.h>
#include <stp/evaluations/global/TheyHaveBallGlobalEvaluation.h>
#include <stp/evaluations/global/WeHaveBallGlobalEvaluation.h>
#include <stp/evaluations/global/WeHaveMajorityGlobalEvaluation.h>

namespace rtt::ai::stp {

uint8_t PlayEvaluator::getGlobalEvaluation(GlobalEvaluation evaluation, const rtt::world::World* world) {
    return (scoresGlobal.contains(evaluation) ? scoresGlobal.at(evaluation) : scoresGlobal[evaluation] = updateGlobalEvaluation(evaluation, world));
}

uint8_t PlayEvaluator::updateGlobalEvaluation(GlobalEvaluation& evaluation, const rtt::world::World* world) {
    auto field = world->getField().value();
    switch (evaluation) {
        case GlobalEvaluation::BallPlacementThemGameState:
            return evaluation::BallPlacementThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallPlacementUsGameState:
            return evaluation::BallPlacementUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::FreeKickThemGameState:
            return evaluation::FreeKickThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::FreeKickUsGameState:
            return evaluation::FreeKickUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::HaltGameState:
            return evaluation::HaltGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::TimeOutGameState:
            return evaluation::TimeOutGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffThemGameState:
            return evaluation::KickOffThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffThemPrepareGameState:
            return evaluation::KickOffThemPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffUsGameState:
            return evaluation::KickOffUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffUsOrNormalGameState:
            return evaluation::KickOffUsOrNormalGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::KickOffUsPrepareGameState:
            return evaluation::KickOffUsPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::NormalOrFreeKickUsGameState:
            return evaluation::NormalOrFreeKickUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::NormalPlayGameState:
            return evaluation::NormalPlayGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyThemGameState:
            return evaluation::PenaltyThemGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyThemPrepareGameState:
            return evaluation::PenaltyThemPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyUsGameState:
            return evaluation::PenaltyUsGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PenaltyUsPrepareGameState:
            return evaluation::PenaltyUsPrepareGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::StopGameState:
            return evaluation::StopGameStateEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallCloseToThem:
            return evaluation::BallCloseToThemGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallCloseToUs:
            return evaluation::BallCloseToUsGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallClosestToUs:
            return evaluation::BallClosestToUsGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallGotShot:
            return evaluation::BallGotShotGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallIsFree:
            return evaluation::BallIsFreeGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallMovesSlow:
            return evaluation::BallMovesSlowGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallOnOurSide:
            return evaluation::BallOnOurSideGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallOnTheirSide:
            return evaluation::BallOnTheirSideGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallInOurDefenseAreaAndStill:
            return evaluation::BallInOurDefenseAreaAndStillGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::BallNotInOurDefenseAreaAndStill:
            return evaluation::BallNotInOurDefenseAreaAndStillGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::DistanceFromBall:
            return evaluation::DistanceFromBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::FreedomOfRobots:
            return evaluation::FreedomOfRobotsGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::GoalVisionFromBall:
            return evaluation::GoalVisionFromBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::GoalVision:
            return evaluation::GoalVisionGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::NoGoalVisionFromBall:
            return evaluation::NoGoalVisionFromBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::WeHaveBall:
            return evaluation::WeHaveBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::TheyHaveBall:
            return evaluation::TheyHaveBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::TheyDoNotHaveBall:
            return evaluation::TheyDoNotHaveBallGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::WeHaveMajority:
            return evaluation::WeHaveMajorityGlobalEvaluation().metricCheck(world, &field);
        case GlobalEvaluation::PreHalfGameState:
            return GameStateManager::getCurrentGameState().getStrategyName() == "formation_pre_half" ? stp::control_constants::FUZZY_TRUE : stp::control_constants::FUZZY_FALSE;
        default:
            RTT_WARNING("Unhandled ScoreEvaluation!");
            return 0;
    }
}
void PlayEvaluator::clearGlobalScores() { scoresGlobal.clear(); }

bool PlayEvaluator::checkEvaluation(GlobalEvaluation globalEvaluation, const rtt::world::World* world, uint8_t cutOff) noexcept {
    return getGlobalEvaluation(globalEvaluation, world) >= cutOff;
}

uint8_t PlayEvaluator::calculateScore(std::vector<PlayScoring>& scoring) {
    double scoreTotal, weightTotal = 0;
    for (auto& factor : scoring) {
        scoreTotal += factor.evaluationScore;
        weightTotal += factor.weight;
    }
    return scoreTotal / weightTotal;
}

}  // namespace rtt::ai::stp