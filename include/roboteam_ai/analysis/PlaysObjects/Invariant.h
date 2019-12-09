//
// Created by jessevw on 06.12.19.
//

#ifndef RTT_INVARIANT_H
#define RTT_INVARIANT_H


#include <include/roboteam_ai/world/World.h>
#include <include/roboteam_ai/world/Field.h>

namespace rtt::ai::analysis {
    class Invariant {
    public:
        Invariant();
        /**
         * @brief base class implementation to check if the invariant is true. Please override in derived classes
         * @param world current world state
         * @param field
         * @return true when the invariant is true, false otherwise
         */
         // TODO: this function should be virtual
        bool isTrue(rtt::ai::world::World* world, rtt::ai::world::Field* field) const;
    };


}


#endif //RTT_INVARIANT_H
