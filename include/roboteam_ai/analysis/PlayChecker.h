//
// Created by jessevw on 04.12.19.
//

#ifndef RTT_PLAYCHECKER_H
#define RTT_PLAYCHECKER_H


#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/Invariant.h"
#include "analysis/PlaysObjects/Play.h"

namespace rtt::ai::analysis {

    class PlayChecker {
    public:
        void constructInvariants();
        PlayChecker() = default;
        bool checkCurrentGameInvariants(rtt::ai::world::World* world, rtt::ai::world::Field* field);
        void update(world::World *world, world::Field *field);

    private:
        /**
         * List of the invariants of the current strategy
         */
        std::vector<std::shared_ptr<Invariant>> invariants;
        /**
         * Vector of all strategies (before pruning)
         */
        std::vector<Play> allPlays;

        std::vector<Play> validPlays;

        /// TODO: implement this function. Not a priority right now
        bool checkStrategyPreconditions();

        void determineNewPlays(world::World *world, world::Field *field);

        /**
         * private typedef to shorten declarations in this class
         */
        using ivec = std::vector<std::shared_ptr<Invariant>>;

    };
}


#endif //RTT_PLAYCHECKER_H
