//
// Created by jessevw on 04.12.19.
//

#include "bt/Composite.hpp"
#include "analysis/PlayChecker.h"

namespace rtt::ai::analysis {
    PlayChecker::PlayChecker() {
        // this->invariants will get default initialized :)
//        this->invariants = {};
    }

    bool PlayChecker::checkCurrentGameInvariants() {
        for (auto invariant : invariants) {
            if (!invariant.isTrue()) {
                determineNewStrategies();
            }
        }
    }



    /**
     * Checks if the strategy
     */
    bool PlayChecker::checkStrategyInvariants() {

    }

    /**
     * Determines what strategies are viable given the current world, ref states and invariants/preconditions
     *
     */
    void PlayChecker::determineNewStrategies() {
        for (auto strategy : allStrategies) {
            strategy.
        }
    }
}


