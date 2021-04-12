//
// Created by maxl on 18-02-21.
//

#ifndef RTT_PLAYEVALUATOR_H
#define RTT_PLAYEVALUATOR_H

#include "world/World.hpp"

namespace rtt::ai::stp{
    enum class GlobalEvaluation{
        /// Game States
        BallPlacementThemGameState = 0,
        BallPlacementUsGameState,
        FreeKickThemGameState,
        FreeKickUsGameState,
        HaltGameState,
        TimeOutGameState,
        KickOffThemGameState,
        KickOffThemPrepareGameState,
        KickOffUsGameState,
        KickOffUsPrepareGameState,
        NormalOrFreeKickUsGameState,
        NormalPlayGameState,
        PenaltyThemGameState,
        PenaltyThemPrepareGameState,
        PenaltyUsGameState,
        PenaltyUsPrepareGameState,
        StopGameState,
        /// Global Evaluations
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
        WeHaveMajority
    };

    class PlayEvaluator {
    public:
        struct PlayScoring {
            uint8_t evaluationScore;
            double weight;
        };

        /**
         * Gets the score of a Global Evaluation, if it was not updated yet, update it before.
         * @param evaluation that needs
         * @return
         */
        uint8_t getGlobalEvaluation(GlobalEvaluation evaluation);

        /**
         * Sets this->world
         * @param world World to update against
         */
        void update(world::World* world) noexcept;

        /**
         * Gets this world, used when a play is not initialised yet (in between plays)
         * @return world
         */
        world::World* getWorld() noexcept;

        /**
         * Make all booleans of updatedGlobal to false (they have not been updated yet this tick)
         */
        void clearGlobalScores();

        /**
         * Checks if FUZZY-TRUE score in uint8-t of global evaluation is above the TRUE threshold
         * @param globalEvaluation Invariant to be checked
         * @param cutOff Bottom bound value of true
         * @return boolean if FUZZY-TRUE is high enough
         */
        bool checkEvaluation(GlobalEvaluation globalEvaluation, uint8_t cutOff = control_constants::FUZZY_DEFAULT_CUTOFF) noexcept;

        /**
         * Calcalute score with the given vector of scores
         * @param scoring vector withto be considered Evaluations
         * @return final score
         */
        uint8_t calculateScore(std::vector<PlayScoring>& scoring);

    private:
        /**
         * Map of all loaded Global Evaluations scores
         */
        std::unordered_map<GlobalEvaluation, uint8_t> scoresGlobal{};


        /**
         * Updates a given Global Evaluation, should only happen if this was not done in this tick yet.
         * @param index of evaluation that needs to be updated
         * @return the score of the updated evaluation
         */
        uint8_t updateGlobalEvaluation(GlobalEvaluation& evaluation);

        /**
         * Current world, do not use before update()
         */
        world::World* world{};

        /**
         *  Current field from world
         */
        world::Field field;
    };

}

#endif //RTT_PLAYEVALUATOR_H
