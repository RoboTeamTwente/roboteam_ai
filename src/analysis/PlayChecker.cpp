//
// Created by jessevw on 04.12.19.
//

#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h>
#include "analysis/PlayChecker.h"
#include "analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h"
#include "analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"
#include "analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h"
#include "analysis/PlaysObjects/Play.h"
#include "functional"


namespace rtt::ai::analysis {

    /**
     * Typedef to facilitate creation of functors for invariant evaluation
     */
    using fun = std::function<bool(world::World*, world::Field*)>;


    fun isAlwaysTrue;
    fun isAlwaysFalse;
    fun ballOnOurSide;
    fun ballBelongsToUs;
    void PlayChecker::constructInvariants() {

        /**
         * Below we declare some lambdas to check if the invariants are true. The left side of the expression is
         * merely a way to store the function that is built. These functions are then given to Plays, and a play
         * is valid when the functions given to it are true.
         */
        isAlwaysTrue = [&](auto world, auto field) {
            bool result = AlwaysTrueInvariant("always false").isTrue(world, field);
            std::cout << "always true " << result << std::endl;
            return result;};
        isAlwaysFalse = [&](auto world, auto field) {
            bool result = AlwaysFalseInvariant("always true").isTrue(world, field);
            std::cout << "always false: " << result << std::endl;
            return result;};
        ballOnOurSide = [&](auto world, auto field) {
            bool result = BallOnOurSideInvariant("always true").isTrue(world, field);
            std::cout << "ball on our side true: " << result << std::endl;
            return result;};
        ballBelongsToUs = [&](auto world, auto field) {
            bool result = BallBelongsToUsInvariant("Ball belongs to us true").isTrue(world, field);
            std::cout << "ball belongs to us true: " << result << std::endl;
            return result;};

    }

    /**
     * Object that stores the current strategy, and checks if the invariants
     * for this play are true. This object is made in the main loop
     * and will be called after the vision data is received, to determine
     * if we need to switch plays (based on whether the invariants of the current play are false).
     * If the one of the invariants is true, the PlayChecker will signal the PlayDecider to recalculate the play,
     * and give it the plays that are possible and allowed.
     */
    PlayChecker::PlayChecker() {
        constructInvariants();
    }

    /**
     * @brief Checks if the invariants of the current play are true for the gamestate
     * @param world
     * @param field
     * @return true if invariants are true, false otherwise
     */
    bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World* world, rtt::ai::world::Field* field) {

        return true;
    }



    /**
     * Determines what plays are viable given the current world, ref states and invariants/preconditions, and stores them in the validPlays vector
     * TODO: add lambda here, to make it faster and cleaner
     */
    void PlayChecker::determineNewPlays(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        std::vector<fun> functionvector = {isAlwaysTrue,  ballOnOurSide, ballBelongsToUs};
        bool result;
        Play p = Play("myPlay", functionvector);
        allPlays.push_back(p);
        for (auto play : allPlays) {
            result = play.isValidPlay(world, field);
            if (result) {
                allPlays.push_back(play);
            }
        }
    }
    /**
     * first check if the current play is still ok
     * if not, loop through every possible play and check for each play if all of its invariants hold
     * Then add these plays to the list of allowed plays
     */
    void PlayChecker::update(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        checkCurrentGameInvariants(world, field);
        determineNewPlays(world, field);

    }

}


