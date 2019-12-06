//
// Created by jessevw on 04.12.19.
//

#include "bt/Composite.hpp"
#include "analysis/PlayChecker.h"
#include "analysis/PlaysObjects/BallBelongsToUs.h"
namespace rtt::ai::analysis {

    /**
     * Object that stores the current strategy, and checks if the invariants
     * for this play are true. This object is made in the main loop
     * and will be called after the vision data is received, to determine
     * if we need to switch plays (based on whether the invariants of the current play are false).
     * If the one of the invariants is true, the PlayChecker will signal the PlayDecider to recalculate the play,
     * and give it the palys that are possible and allowed.
     */
    PlayChecker::PlayChecker() {
        // this->invariants will get default initialized :)
        this->invariants = {rtt::ai::analysis::BallBelongsToUs()};
        this->currentPlay = myPlay();
    }

    /**
     * @brief Checks
     * @param world
     * @param field
     * @return
     */
    bool PlayChecker::checkCurrentGameInvariants(rtt::ai::world::World* world, rtt::ai::world::Field* field) {
        return std::all_of(invariants.begin(), invariants.end(),
                [&](auto const& inv){ return inv.isTrue(world, field); });

        for (auto const invariant : invariants) {
            if (!invariant.isTrue(world, field)) {
                return false;
            }
        }
        return true;
    }


    /**
     * Checks if the play's invariants are true for the current game state
     */
    bool PlayChecker::checkStrategyInvariants() {

    }

    /**
     * Checks if the play's preconditions are true for the current game state
     * @return true if the play's precondition are true, false otherwise
     */
    bool PlayChecker::checkStrategyPreconditions() {

    }

    /**
     * Determines what plays are viable given the current world, ref states and invariants/preconditions
     */
    void PlayChecker::determineNewPlays() {
        for (auto strategy : allStrategies) {
        }
    }
}


