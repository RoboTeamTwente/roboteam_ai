#include "UntilFail.hpp"

namespace bt {

Node::Status UntilFail::Update() {
  Node::append_status("[UntilFail: executing child of type %s]", child->node_name().c_str());
  auto status = child->Tick();

  if (status==Status::Failure) {
    return Status::Success;
  } else if (status==Status::Invalid) {
    return Status::Failure;
  } else /* if (status == Status::Running || status == Status::Success) */ {
    // If the status was anything else, we just keep running
    return Status::Running;
  }
}

std::string UntilFail::node_name() {
  return "UntilFail";
}

} // bt
