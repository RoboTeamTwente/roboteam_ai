#include "MemSelector.hpp"

namespace bt {
void MemSelector::Initialize() {
  index = 0;
}

Node::Status MemSelector::Update() {
  if (HasNoChildren()) {
    return Status::Success;
  }

  // Keep going until a child behavior says it's running.
  while (index < children.size()) {
    auto &child = children.at(index);
    Node::append_status("[MemSelector: executing child of type %s]", child->node_name().c_str());
    auto status = child->Tick();

    // If the child succeeds, or keeps running, do the same.
    if (status!=Status::Failure) {
      return status;
    }

    index++;
  }

  return Status::Failure;
}

using Ptr = std::shared_ptr<MemSelector>;

std::string MemSelector::node_name() {
  return "MemSelector";
}

} //bt
