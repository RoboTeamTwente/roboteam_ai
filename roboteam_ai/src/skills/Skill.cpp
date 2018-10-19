#include "Skill.h"

namespace rtt {
namespace ai {

Skill::Skill(std::string name, bt::Blackboard::Ptr blackboard)
    : bt::Leaf(name, blackboard) {}

bt::Node::Status Skill::Update() {
  return Status::Invalid;
}

} // ai
} // rtt