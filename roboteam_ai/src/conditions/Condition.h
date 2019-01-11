#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"

namespace rtt {
namespace ai {

//forward declare control utils
namespace control {
    class ControlUtils;
}

class Condition : public bt::Leaf {
    public:
        using Control = control::ControlUtils;
        using Status = bt::Node::Status;

        explicit Condition(std::string name, bt::Blackboard::Ptr blackboard = nullptr);
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
