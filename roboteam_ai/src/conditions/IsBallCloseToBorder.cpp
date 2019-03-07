//
// Created by robzelluf on 2/18/19.
//

#include "IsBallCloseToBorder.h"

namespace rtt{
namespace ai {

IsBallCloseToBorder::IsBallCloseToBorder(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(std::move(name), std::move(blackboard)) { };

void IsBallCloseToBorder::initialize() {
    if (properties->hasDouble("margin")) {
        margin = properties->getDouble("margin");
    } else {
        margin = Constants::CLOSE_TO_BORDER_DISTANCE();
    }
    layingStill = properties->getBool("layingStill");
}

bt::Node::Status IsBallCloseToBorder::update() {
    Vector2 ballPos = World::getBall()->pos;
    if (!properties->getBool("corner")) {
        if (Field::pointIsInField(ballPos, static_cast<float>(margin))) {
            return Status::Failure;
        }
    } else {
        roboteam_msgs::GeometryFieldSize field = Field::get_field();
        double xDiff = field.field_length / 2 - abs(ballPos.x);
        double yDiff = field.field_width / 2 - abs(ballPos.y);

        if (xDiff >= margin || yDiff >= margin) {
            return Status::Failure;
        }
    }

    if (!layingStill) {
        std::cout << "LOL1" << std::endl;
        return Status::Success;
    } else if (Vector2(ball->vel).length() <= Constants::BALL_STILL_VEL()) {
        return Status::Success;
    } else {
        return Status::Failure;
    }
}

std::string IsBallCloseToBorder::node_name() {return "IsBallCloseToBorder";}

}
}
