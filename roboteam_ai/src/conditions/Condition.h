#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"
#include "../utilities/Field.h"
#include "../utilities/World.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

/**
 * \class Condition
 * \brief Base class for conditions.
 */
class Condition : public bt::Leaf {
    public:
        using Control = control::ControlUtils;
        using Status = bt::Node::Status;

        explicit Condition(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
