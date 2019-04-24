//
// Created by baris on 10-4-19.
//

#include "RoleDivider.h"

namespace bt {

void RoleDivider::addChild(bt::Node::Ptr node) {
    children.emplace_back(node);
}

std::vector<Node::Ptr> RoleDivider::getChildren() {
    return children;
}

std::string RoleDivider::node_name() {
    return name;
}

void RoleDivider::giveProperty(std::string a, std::string b) {
    properties->setString(a, b);
}

void RoleDivider::initialize() {
}

Node::Status RoleDivider::update() {
    return Status::Waiting;
}

void RoleDivider::terminate(Node::Status s) {
}

}