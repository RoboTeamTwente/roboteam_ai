#include "Node.hpp"
#include "BehaviorTree.hpp"

namespace bt {

BehaviorTree::BehaviorTree() : blackboard(std::make_shared<Blackboard>()) {

}

BehaviorTree::BehaviorTree(const Node::Ptr &rootNode) : BehaviorTree() {
  root = rootNode;
}

BehaviorTree::BehaviorTree(const Blackboard::Ptr &shared) : BehaviorTree() {
  sharedBlackboard = shared;
}

BehaviorTree::~BehaviorTree() {

}

Node::Status BehaviorTree::Update() {
  return root->Tick();
}

void BehaviorTree::Terminate(Status s) {
  if (root->getStatus()==Status::Running) {
    root->Terminate(root->getStatus());
  }

  if (s==Node::Status::Running) {
    setStatus(Node::Status::Failure);
  }
}

void BehaviorTree::SetRoot(const Node::Ptr &node) {
  root = node;
}

Node::Ptr BehaviorTree::GetRoot() {
  return root;
}

Blackboard::Ptr BehaviorTree::GetBlackboard() const {
  return blackboard;
}

Blackboard::Ptr BehaviorTree::GetSharedBlackboard() const {
  return sharedBlackboard;
}

void BehaviorTree::SetSharedBlackboard(const Blackboard::Ptr &shared) {
  sharedBlackboard = shared;
}

}
