#pragma once

#include "Node.hpp"

namespace bt {

class Decorator : public Node {
 public:
  virtual ~Decorator();

  void AddChild(Node::Ptr child) override;
  bool HasNoChild() const;

  void Terminate(Status s) override;

 protected:
  Node::Ptr child = nullptr;
};

} // bt
