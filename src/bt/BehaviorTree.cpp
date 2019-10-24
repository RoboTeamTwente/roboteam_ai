#include "bt/Node.hpp"
#include "bt/BehaviorTree.hpp"

namespace bt {

BehaviorTree::BehaviorTree() {
    globalBB = std::make_shared<Blackboard>();
}
BehaviorTree::BehaviorTree(std::string name) {
    globalBB = std::make_shared<Blackboard>();
    this->name = name;
}
BehaviorTree::BehaviorTree(const Node::Ptr &rootNode)
        :BehaviorTree() {
    root = rootNode;
    this->root->globalBB = globalBB;
}

BehaviorTree::BehaviorTree(const Blackboard::Ptr &shared)
        :BehaviorTree() {
    globalBB = shared;
}


Node::Status BehaviorTree::update() {
    std::cout << "tree being updated:" + name << std::endl;
    if(name == "defendertree"){
        std::cout << "this makes me really happy" << std::endl;
    }
    return root->tick();
}

void BehaviorTree::terminate(Status s) {
    if (root->getStatus() == Status::Running) {
        root->terminate(root->getStatus());
    }

    if (s == Node::Status::Running) {
        setStatus(Node::Status::Failure);
    }
}

void BehaviorTree::SetRoot(const Node::Ptr &node) {
    root = node;
    node->globalBB = globalBB;
}

Node::Ptr BehaviorTree::GetRoot() {
    return root;
}

void BehaviorTree::SetGlobalBlackboard(const Blackboard::Ptr &global) {
    globalBB = global;
}
std::string BehaviorTree::node_name() {
    return "Behaviour Tree";
}

}
