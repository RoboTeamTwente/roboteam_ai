//
// Created by jessevw on 04.12.19.
//

#ifndef RTT_PLAYCHECKER_H
#define RTT_PLAYCHECKER_H


#include "analysis/PlaysObjects/Invariant.h"
#include "analysis/PlaysObjects/Play.h"

namespace rtt::ai::analysis {
    class PlayChecker {
    public:
        PlayChecker();
        bool checkCurrentGameInvariants(rtt::ai::world::World* world, rtt::ai::world::Field* field);
        bool checkStrategyInvariants();
        void determineNewPlays();

    private:
        /**
         * List of the invariants of the current strategy
         */
        std::vector<rtt::ai::analysis::Invariant> invariants;
        /**
         * Vector of all strategies (before pruning)
         */
        std::vector<std::string> allStrategies;

        Play currentPlay;

        bool checkStrategyPreconditions();
    };
}


#endif //RTT_PLAYCHECKER_H
