#include "bt/Node.h"
#include <iostream>
#include <memory>
#include <roboteam_utils/Print.h>
#include "bt/Blackboard.h"

namespace bt {

void Node::initialize() {}

void Node::terminate(Status s) {
    init = false;
    // If we're terminating while we're still running,
    // consider the node failed. If it already failed
    // or succeeded, leave it like that.
    if (s == Status::Running) {
        status = Status::Failure;
    }
}

Node::Status Node::tick(rtt::ai::world::World *world, const rtt::ai::world::Field *field) {
    this->world = world;
    this->field = field;

    amountOfTicks++;
    //    lastTickTime = ros::Time::now();

    if (!init) {
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

std::string Node::node_name() { return "Node name undefined node.cpp"; }

void Node::addChild(bt::Node::Ptr) {}

// testing purpose
std::vector<Node::Ptr> Node::getChildren() {
    // std::cerr << "Get children in Node.cpp, should never be called" << std::endl;
    return std::vector<Node::Ptr>{};
}

void Node::setProperties(bt::Blackboard::Ptr blackboard) { properties = blackboard; }
Node::Node() { globalBB = std::make_shared<Blackboard>(); }

Node::Status Node::NodeUpdate() {
    status = update();
    return status;
}

void Node::NodeInitialize() {
    initialize();
    init = true;
}

void Node::NodeTerminate(Status s) {
    terminate(s);
    init = false;
}

unsigned long long Node::getAmountOfTicks() const { return amountOfTicks; }

std::string Node::status_print(Node::Status s) {
    switch (s) {
        case Status::Waiting:
            return "Waiting";
        case Status::Success:
            return "Success";
        case Status::Failure:
            return "Failure";
        case Status::Running:
            return "Running";
        default:
            std::cout << "Enum failure in Node::Status status_print for node: " << node_name() << std::endl;
            return "ERROR!!";
    }
}

void Node::setRoleString(std::string roleName) {
    auto children = this->getChildren();
    if (properties) {
        this->properties->setString("ROLE", roleName);
    }

    for (const auto& child : this->getChildren()) {
        if (child) {
            child->setRoleString(roleName);
        }
    }
}
void Node::giveProperty(std::string a, std::string b) {
    RTT_ERROR("calling giveProperty() function to base class Node")
}

}  // namespace bt
