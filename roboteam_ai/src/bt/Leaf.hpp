#pragma once

#include "Node.hpp"
#include "Blackboard.hpp"
#include <memory>

namespace bt {

class Leaf : public Node {
 public:
  Leaf();
  virtual ~Leaf();
  Leaf(Blackboard::Ptr blackboard);
  void SetBlackboard(Blackboard::Ptr blackboard);

  virtual Status Update() = 0;

 protected:
  Blackboard::Ptr blackboard;
};

}
