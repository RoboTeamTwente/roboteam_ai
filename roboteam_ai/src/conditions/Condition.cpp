#include "Condition.h"

namespace rtt {
namespace ai {

Condition::Condition(std::string name, bt::Blackboard::Ptr blackboard)
        :bt::Leaf(name, blackboard) { }

} // ai
} // rtt