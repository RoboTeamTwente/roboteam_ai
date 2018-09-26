#include "Decorator.hpp"

namespace bt {

Decorator::~Decorator() {}

void Decorator::SetChild(Node::Ptr child) {
  this->child = child;
}

bool Decorator::HasNoChild() const {
  return child==nullptr;
}

void Decorator::Terminate(Status s) {
  if (child->getStatus()==Status::Running) {
    child->Terminate(child->getStatus());
  }

  if (s==Status::Running) {
    setStatus(Status::Failure);
  }
}

} // bt
