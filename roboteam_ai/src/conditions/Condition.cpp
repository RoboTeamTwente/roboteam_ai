#include "Condition.h"

namespace rtt {
namespace ai {

Condition::Condition(std::string name, bt::Blackboard::Ptr blackboard)
    : bt::Leaf(name, blackboard) {}

bt::Node::Status Condition::Update() {
  return Status::Invalid;
}

} // ai
} // rtt