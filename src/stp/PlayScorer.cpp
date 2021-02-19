//
// Created by maxl on 18-02-21.
//

#include <stp/PlayScorer.h>
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
    uint8_t Score::getGlobalEvaluation(GlobalEvaluation evaluation) {
        return (updatedGlobal[(int) evaluation]) ? scoresGlobal[(int) evaluation] : scoresGlobal[(int) evaluation] = updateGlobalEvaluation(evaluation);
    }

    uint8_t Score::updateGlobalEvaluation(GlobalEvaluation evaluation) {
        updatedGlobal[(int) evaluation] = true;
        switch (evaluation) {
            case GlobalEvaluation::BallCloseToThem:
                return invariant::BallCloseToThemInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallCloseToUs:
                return invariant::BallCloseToUsInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallClosestToUs:
                return invariant::BallClosestToUsInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallGotShot:
                return invariant::BallGotShotInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallIsFree:
                return invariant::BallIsFreeInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallMovesSlow:
                return invariant::BallMovesSlowInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallOnOurSide:
                return invariant::BallOnOurSideInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallOnTheirSide:
                return invariant::BallOnTheirSideInvariant().metricCheck(world, field);
            case GlobalEvaluation::BallShotOrCloseToThem:
                return invariant::BallShotOrCloseToThemInvariant().metricCheck(world, field);
            case GlobalEvaluation::DistanceFromBall:
                return invariant::DistanceFromBallInvariant().metricCheck(world, field);
            case GlobalEvaluation::FreedomOfRobots:
                return invariant::FreedomOfRobotsInvariant().metricCheck(world, field);
            case GlobalEvaluation::GoalVisionFromBall:
                return invariant::GoalVisionFromBallInvariant().metricCheck(world, field);
            case GlobalEvaluation::GoalVision:
                return invariant::GoalVisionInvariant().metricCheck(world, field);
            case GlobalEvaluation::NoGoalVisionFromBall:
                return invariant::NoGoalVisionFromBallInvariant().metricCheck(world, field);
            case GlobalEvaluation::WeHaveBall:
                return invariant::WeHaveBallInvariant().metricCheck(world, field);
            case GlobalEvaluation::WeHaveMajority:
                return invariant::WeHaveMajorityInvariant().metricCheck(world, field);
            default:
                RTT_WARNING("Unhandled ScoreEvaluation!");
                return 0;
        }
    }

    void Score::clearGlobalScores() {
        for (int i = 0; i < sizeof(GlobalEvaluation); i++) updatedGlobal[i] = false;
    }
}