#pragma once

#include "Node.hpp"

namespace bt {

class Decorator : public Node {
 public:
  virtual ~Decorator();

  void SetChild(Node::Ptr child);
  bool HasNoChild() const;

  void Terminate(Status s) override;

 protected:
  Node::Ptr child = nullptr;
};

} // bt
