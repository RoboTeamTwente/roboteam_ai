#include <memory>

#include "Blackboard.hpp"
#include "Node.hpp"

namespace bt {

Node::~Node() {

}

void Node::Initialize() {

}

void Node::Terminate(Status s) {
  // If we're terminating while we're still running,
  // consider the node failed. If it already failed
  // or succeeded, leave it like that.
  if (s==Status::Running) {
    status = Status::Failure;
  }
}

Node::Status Node::Tick() {
  if (status!=Status::Running) {
    Initialize();
  }

  status = Update();

  if (status!=Status::Running) {
    Terminate(status);
  }

  return status;
}

bool Node::IsSuccess() const { return status==Status::Success; }
bool Node::IsFailure() const { return status==Status::Failure; }
bool Node::IsRunning() const { return status==Status::Running; }
bool Node::IsTerminated() const { return IsSuccess() || IsFailure(); }
Node::Status Node::getStatus() const { return status; }
void Node::setStatus(Status s) { status = s; }

std::string Node::node_name() {
  return "<ERROR>";
}

void Node::append_status(std::string fmt, ...) {
  char buf[1024];
  va_list varargs;
  va_start(varargs, fmt);
  vsnprintf(buf, 1024, fmt.c_str(), varargs);
  va_end(varargs);

  status_desc += std::string(buf);
}

std::string statusToString(bt::Node::Status status) {
  if (status==bt::Node::Status::Success) {
    return "Success";
  } else if (status==bt::Node::Status::Failure) {
    return "Failure";
  } else if (status==bt::Node::Status::Invalid) {
    return "Invalid";
  } else if (status==bt::Node::Status::Running) {
    return "Running";
  } else {
    std::cout << "Enum failure in Node::Status overload of to_string\n";
    return "ERROR ERROR!!!";
  }
}

}
