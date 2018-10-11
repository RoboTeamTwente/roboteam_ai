#pragma once

#include "Node.hpp"

namespace bt {

class Composite : public Node {
 public:
  virtual ~Composite();

  virtual void AddChild(Node::Ptr child);
  bool HasNoChildren() const;
  void Terminate(Status s) override;

 protected:
  Nodes children;
  size_t index = 0;
};

}
