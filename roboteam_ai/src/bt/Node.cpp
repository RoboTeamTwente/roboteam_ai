#include <memory>
#include <iostream>

#include "Blackboard.hpp"
#include "Node.hpp"

namespace bt {

void Node::initialize() {

}

void Node::terminate(Status s) {
    // If we're terminating while we're still running,
    // consider the node failed. If it already failed
    // or succeeded, leave it like that.
    if (s == Status::Running) {
        status = Status::Failure;
    }
}

Node::Status Node::tick() {
    if (status != Status::Running) {
        NodeInitialize();
    }

    setStatus(NodeUpdate());

    if (status != Status::Running) {
        NodeTerminate(status);
    }

    return status;
}

bool Node::IsSuccess() const { return status == Status::Success; }

bool Node::IsFailure() const { return status == Status::Failure; }

bool Node::IsRunning() const { return status == Status::Running; }

bool Node::IsTerminated() const { return IsSuccess() || IsFailure(); }

Node::Status Node::getStatus() const { return status; }

void Node::setStatus(Status s) { status = s; }

std::string Node::node_name() {
    return "Node name undefined node.cpp";
}

void Node::addChild(bt::Node::Ptr) { }

// testing purpose
std::vector<Node::Ptr> Node::getChildren() {
    return std::vector<Node::Ptr>{};
}

void Node::setProperties(bt::Blackboard::Ptr blackboard) {
    properties = blackboard;

}
Node::Node() {
    globalBB = std::make_shared<Blackboard>();
}

Node::Status Node::NodeUpdate() {
    auto status = update();
    //std::cout << "Node Update:  " << node_name() << status_print(status) << std::endl;

    return status;
}

void Node::NodeInitialize() {
    //std::cout << "Node Initialize:  " << node_name() << std::endl;
    initialize();
}

void Node::NodeTerminate(Status s) {
    //std::cout << "Node Terminate:  " << node_name() << std::endl;
    terminate(s);
}

std::string statusToString(bt::Node::Status status) {
    if (status == bt::Node::Status::Success) {
        return "Success";
    }
    else if (status == bt::Node::Status::Failure) {
        return "Failure";
    }
    else if (status == bt::Node::Status::Waiting) {
        return "Waiting";
    }
    else if (status == bt::Node::Status::Running) {
        return "Running";
    }
    else {
        std::cout << "Enum failure in Node::Status overload of to_string\n";
        return "ERROR ERROR!!!";
    }
}

}
