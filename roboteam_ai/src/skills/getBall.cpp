//
// Created by rolf on 04/12/18.
//

#include "getBall.h"
namespace rtt {
namespace ai {
GetBall::GetBall(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}
std::string GetBall::node_name() {
    return "GetBall";
}
void GetBall::initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("GetBall Initialize -> robot does not exist in world");
            return;
        }
    }

    else {
        ROS_ERROR("GetBall Initialize -> ROLE WAITING!!");
        return;
    }
}
GetBall::Status GetBall::update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    }
    else {
        ROS_ERROR("GetBall Update -> robot does not exist in world");
    }
}
void GetBall::terminate(Status s) {

}

}//rtt
}//ai