//
// Created by Rolf on 17-10-18.
//
#include "IsInDefenseArea.hpp"

namespace rtt {
namespace ai {

IsInDefenseArea::IsInDefenseArea(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {

}

bt::Node::Status IsInDefenseArea::update() {
    Vector2 point;
    if (properties->getBool("useRobot")) {
        robot = getRobotFromProperties(properties);
        if (robot) {
            point = robot->pos;
        }
        else{
            return Status::Failure;
        }
    }
    else{
        //TODO: Fix this once ball PR is merged.
        auto ball=World::getBall();
        point=ball.pos;
    }
    ourDefenseArea = properties->getBool("ourDefenseArea");
    if (properties->hasDouble("margin")) margin = static_cast<float>(properties->getDouble("margin"));
    else margin = 0.0f;

    roboteam_msgs::World world = World::get_world();
    if (Field::pointIsInDefenceArea(point, ourDefenseArea, margin)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
