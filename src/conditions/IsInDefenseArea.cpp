/*
 * Returns SUCCESS if the position is within the defence area.
 * properties:
 * - useRobot: if true, use the robot location, otherwise use the ball location
 * - ourDefenceArea: if true, the condition returns true if the point is in our defence area. otherwise it is their defence area
 * - outsideField: if true, the robot
 */

#include "conditions/IsInDefenseArea.hpp"

namespace rtt::ai {

IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard) : Condition(std::move(name), std::move(blackboard)) {}

bt::Node::Status IsInDefenseArea::onUpdate() {
    ourDefenseArea = properties->getBool("ourDefenseArea");
    outsideField = properties->getBool("outsideField");
    secondsAhead = properties->hasDouble("secondsAhead") ? properties->getDouble("secondsAhead") : 0.0;
    if (secondsAhead >= 0.0) {
        if (properties->getBool("useRobot")) {
            point = robot->get()->getPos() + robot->get()->getVel() * secondsAhead;
        } else {
            point = ball->get()->getPos() + ball->get()->getVelocity() * secondsAhead;
        }
    } else {
        point = properties->getBool("useRobot") ? robot->get()->getPos() : ball->get()->getPos();
    }

    margin = properties->hasDouble("margin") ? static_cast<float>(properties->getDouble("margin")) : 0.0f;

    if (FieldComputations::pointIsInDefenceArea(*field, point, ourDefenseArea, margin, outsideField)) {
        return Status::Success;
    }
    return Status::Failure;
}

}  // namespace rtt::ai
