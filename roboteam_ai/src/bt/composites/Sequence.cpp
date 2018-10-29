#include "Sequence.hpp"

namespace bt {

Node::Status Sequence::Update() {
  if (HasNoChildren()) {
    return Status::Success;
  }

  // Keep going until a child behavior says it's running.
  for (auto &child : children) {
    Node::append_status("[Sequence: executing child of type %s]", child->node_name().c_str());
    auto status = child->Tick();

    // If the child fails, or keeps running, do the same.
    if (status!=Status::Success) {
      return status;
    }
  }

  return Status::Success;
}

std::string Sequence::node_name() {
  return "Sequence";
}

} // bt
