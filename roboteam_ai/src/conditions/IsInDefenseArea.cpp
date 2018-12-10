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


    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = * World::getRobotForId(robot.id, true);
        }
        else {
            ROS_ERROR("HasBall Update -> robot does not exist in world");
            return Status::Failure;
        }
    }
    else {
        ROS_ERROR("HasBall Update -> ROLE WAITING!!");
        return Status::Failure;
    }

    ourDefenseArea = properties->getBool("ourDefenseArea");
    if (properties->hasDouble("margin")) margin = static_cast<float>(properties->getDouble("margin"));
    else margin = 0.0f;

    roboteam_msgs::World world = World::get_world();

    if (Field::pointIsInDefenceArea(robot.pos, ourDefenseArea, margin)) {
        return Status::Success;
    }
    return Status::Failure;
}

} // ai
} // rtt
