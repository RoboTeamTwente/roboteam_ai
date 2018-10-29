#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"
#include "../utilities/Field.h"
#include "../utilities/World.h"

namespace rtt {
    namespace ai {

/**
 * \class Condition
 * \brief Base class for conditions.
 */
        class Condition : public bt::Leaf {
        public:
            explicit Condition(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);

            Status Update() override;
        };

    } // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
