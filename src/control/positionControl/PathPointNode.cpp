//
// Created by ratoone on 20-02-20.
//

#include "control/positionControl/PathPointNode.h"

namespace rtt::ai::control {
PathPointNode::PathPointNode(const Vector2 &position) : position(position) {}

PathPointNode::PathPointNode(const Vector2 &position, PathPointNode &parent) : position{position}, parent{&parent} {}

const Vector2 &PathPointNode::getPosition() const { return position; }

PathPointNode *PathPointNode::getParent() const { return parent; }

}  // namespace rtt::ai::control