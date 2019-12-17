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
        /**
         * @brief base class implementation for invariants. Non virtual inheritance, mainly to save time rewriting base functions of derived invariant classes
         * @param world current world state
         * @param field
         * @return true when the invariant is true, false otherwise
         */
        const std::string getName();

    protected:
        std::string name;
    };


}


#endif //RTT_INVARIANT_H
