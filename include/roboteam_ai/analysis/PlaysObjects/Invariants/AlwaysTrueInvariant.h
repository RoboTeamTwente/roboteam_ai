//
// Created by jessevw on 11.12.19.
//

#ifndef RTT_ALWAYSTRUEINVARIANT_H
#define RTT_ALWAYSTRUEINVARIANT_H

#include "analysis/PlaysObjects/Invariants/Invariant.h"

namespace rtt::ai::analysis {
    /**
     * Invariant that is true when the ball belongs to us
     */
    class AlwaysTrueInvariant : public Invariant {
    private:
        std::string name;
    public:
        /**
         * Functional implementation of when the ball belongs to us
         * @param world the current world state
         * @param field
         * @return true if the ball belongs to us, false otherwise
         */
        static bool isValid(rtt::ai::world::World *world, rtt::ai::world::Field *field);

        AlwaysTrueInvariant(std::string name);

    };
}


#endif //RTT_ALWAYSTRUEINVARIANT_H
