#include "UntilSuccess.hpp"

namespace bt {

Node::Status UntilSuccess::Update() {
  Node::append_status("[UntilSuccess: executing child of type %s]", child->node_name().c_str());
  auto status = child->Tick();

  if (status==Status::Success) {
    return Status::Success;
  } else if (status==Status::Invalid) {
    return Status::Failure;
  } else /* if (status == Status::Failure || status == Status::Running) */ {
    // If the status was anything but success/invalid, keep running
    return Status::Running;
  }
}

std::string UntilSuccess::node_name() {
  return "UntilSuccess";
}

} // bt
