#include <memory>

#include "Node.hpp"
#include "Leaf.hpp"

namespace bt {

Leaf::Leaf() {}

Leaf::Leaf(Blackboard::Ptr blackboard) : blackboard(blackboard) {}

Leaf::~Leaf() {}

void Leaf::SetBlackboard(Blackboard::Ptr blackboard) {
  this->blackboard = blackboard;
}

}
