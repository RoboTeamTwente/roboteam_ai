//
// Created by maxl on 18-02-21.
//

#ifndef RTT_PLAYSCORER_H
#define RTT_PLAYSCORER_H

#include "world/Field.h"
#include "world/views/WorldDataView.hpp"

namespace rtt::ai::stp{
    enum class GlobalEvaluation{
        BallCloseToThem,
        BallCloseToUs,
        BallClosestToUs,
        BallGotShot,
        BallIsFree,
        BallMovesSlow,
        BallOnOurSide,
        BallOnTheirSide,
        BallShotOrCloseToThem,
        DistanceFromBall,
        FreedomOfRobots,
        GoalVisionFromBall,
        GoalVision,
        NoGoalVisionFromBall,
        WeHaveBall,
        WeHaveMajority,
    };

    class Score {
    public:
        uint8_t getGlobalEvaluation(GlobalEvaluation evaluation);
        void clearGlobalScores();

    private:
        rtt::world::view::WorldDataView world;
        rtt::world::Field *field;

        std::array<uint8_t, sizeof(GlobalEvaluation)> scoresGlobal;
        std::array<bool, sizeof(GlobalEvaluation)> updatedGlobal;

        uint8_t updateGlobalEvaluation(GlobalEvaluation evaluation);
    };

}

#endif //RTT_PLAYSCORER_H
