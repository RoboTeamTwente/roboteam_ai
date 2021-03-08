//
// Created by maxl on 18-02-21.
//

#ifndef RTT_PLAYSCORER_H
#define RTT_PLAYSCORER_H

#include "world/World.hpp"

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

    class PlayScorer {
    public:
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


    private:
        /**
         * Array of all Global Evaluations scores
         */
        std::array<uint8_t, sizeof(GlobalEvaluation)> scoresGlobal{};

        /**
         * Array of all Global Evaluations booleans if they have been updated
         */
        std::array<bool, sizeof(GlobalEvaluation)> updatedGlobal{};

        /**
         * Updates a given Global Evaluation, should only happen if this was not done in this tick yet.
         * @param index of evaluation that needs to be updated
         * @return the score of the updated evaluation
         */
        uint8_t updateGlobalEvaluation(int index);

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

#endif //RTT_PLAYSCORER_H
