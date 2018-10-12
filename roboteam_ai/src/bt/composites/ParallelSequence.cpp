#include "ParallelSequence.hpp"

namespace bt {

ParallelSequence::ParallelSequence(bool successOnAll, bool failOnAll)
    : useSuccessFailPolicy(true), successOnAll(successOnAll), failOnAll(failOnAll) {}
ParallelSequence::ParallelSequence(int minSuccess, int minFail) : minSuccess(minSuccess), minFail(minFail) {}

bt::Node::Status ParallelSequence::Update() {
  int minimumSuccess = minSuccess;
  int minimumFail = minFail;

  if (useSuccessFailPolicy) {
    if (successOnAll) {
      minimumSuccess = children.size();
    } else {
      minimumSuccess = 1;
    }

    if (failOnAll) {
      minimumFail = children.size();
    } else {
      minimumFail = 1;
    }
  }

  int totalSuccess = 0;
  int totalFail = 0;
  for (auto &child : children) {
    Node::append_status("[Parallel: executing child of type %s]", child->node_name().c_str());
    auto status = child->Tick();
    if (status==Status::Success) {
      totalSuccess++;
    }
    if (status==Status::Failure) {
      totalFail++;
    }
  }

  if (totalSuccess >= minimumSuccess) {
    return Status::Success;
  }
  if (totalFail >= minimumFail) {
    return Status::Failure;
  }

  return Status::Running;
}

std::string ParallelSequence::node_name() { return "ParallelSequence"; }

}
