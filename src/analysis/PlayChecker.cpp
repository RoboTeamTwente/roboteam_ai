//
// Created by jessevw on 04.12.19.
//

#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/AlwaysTrueInvariant.h>
#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/BallOnOurSideInvariant.h>
#include <include/roboteam_ai/analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h>
#include "analysis/PlayChecker.h"
#include "analysis/PlaysObjects/Invariants/AlwaysFalseInvariant.h"
#include "functional"


namespace rtt::ai::analysis {
    /**
     * Typedef to facilitate creation of functors for invariant evaluation
     */
    using fun = std::function<bool(world::World *, world::Field *)>;

    std::vector<fun> x = {
            [](auto world, auto field) { return AlwaysTrueInvariant().isTrue(world, field); },
    };


    enum invariantEnum {
        alwaysTrueIndex,
        alwaysFalseIndex,
    };

//    x[alwaysTrueIndex]


    void PlayChecker::constructInvariants() {

        isAlwaysTrue = [&](auto world, auto field) { return AlwaysTrueInvariant().isTrue(world, field); };
        isAlwaysFalse = [&](auto world, auto field) { return AlwaysFalseInvariant().isTrue(world, field); };
        ballOnOurSide = [&](auto world, auto field) { return BallOnOurSideInvariant().isTrue(world, field); };
        ballBelongsToUs = [&](auto world, auto field) { return BallBelongsToUsInvariant().isTrue(world, field); };
    }


    PlayChecker::PlayChecker() {
        constructInvariants();
    }

    /**
     * Object that stores the current strategy, and checks if the invariants
     * for this play are true. This object is made in the main loop
     * and will be called after the vision data is received, to determine
     * if we need to switch plays (based on whether the invariants of the current play are false).
     * If the one of the invariants is true, the PlayChecker will signal the PlayDecider to recalculate the play,
     * and give it the plays that are possible and allowed.
     */


    /**
     * @brief Checks if the invariants of the current play are true for the gamestate
     * @param world
     * @param field
     * @return true if invariants are true, false otherwise
     */
    bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        std::cout << "checking if the play is still valid" << std::endl;
        std::cout << "Invariants name:";
        return true;
    }


    /**
     * Determines what plays are viable given the current world, ref states and invariants/preconditions, and stores them in the validPlays vector
     * TODO: add lambda here, to make it faster and cleaner
     */
    void PlayChecker::determineNewPlays(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        Play p{};

        std::vector<fun> functionvector = {isAlwaysFalse, isAlwaysTrue};
        p.setInvariants(functionvector);

        bool result = false;
        for (auto const &play : allPlays) {
            play.isValidPlay(world, field);
            std::cout << result << std::endl;
        }
    }

    /**
     * first check if the current play is still ok
     * if not, loop through every possible play and check for each play if all of its invariants hold
     * Then add these plays to the list of allowed plays
     */
    void PlayChecker::update(rtt::ai::world::World *world, rtt::ai::world::Field *field) {
        if (!checkCurrentGameInvariants(world, field)) {
            determineNewPlays(world, field);
        }

    }
}


