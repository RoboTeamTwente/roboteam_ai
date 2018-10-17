#include "Selector.hpp"

namespace bt {

Node::Status Selector::Update() {
  // Keep going until a child behavior says it's running.
  for (auto &child : children) {
    Node::append_status("[Selector: executing child of type %s]", child->node_name().c_str());
    auto status = child->Tick();

    // If the child succeeds, or keeps running, do the same.
    if (status!=Status::Failure) {
      return status;
    }
  }

  return Status::Failure;
}

std::string Selector::node_name() {
  return "Selector";
}

} // bt
