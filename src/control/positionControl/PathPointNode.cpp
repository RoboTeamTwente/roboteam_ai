//
// Created by ratoone on 20-02-20.
//

#include "control/positionControl/PathPointNode.h"

namespace rtt::ai::control{
PathPointNode::PathPointNode(const Vector2& position) : position{position}{

}

const Vector2 &PathPointNode::getPosition() const {
    return position;
}

double PathPointNode::getTime() const {
    return time;
}

PathPointNode *PathPointNode::getParent() const {
    return parent;
}

void PathPointNode::setParent(PathPointNode &parent) {
    this->parent = &parent;
}
}