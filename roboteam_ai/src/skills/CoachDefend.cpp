//
// Created by rolf on 5-3-19.
//

#include "CoachDefend.h"
#include "../utilities/DefensiveCoach.h"

namespace rtt{
namespace ai{
CoachDefend::CoachDefend(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void CoachDefend::onInitialize() {
coach::DefensiveCoach::addDefender(robot->id);
}


bt::Node::Status CoachDefend::onUpdate() {
    auto targetLocation = coach::DefensiveCoach::getDefenderPosition(robot->id);
    if (!targetLocation){
        ROS_ERROR("Could not find the location of defender in calculated positions!");
        return bt::Node::Status::Failure;
    }
    auto velocities = gtp.goToPos(robot, *targetLocation, control::GoToType::clean);

    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(velocities.x);
    cmd.y_vel = static_cast<float>(velocities.y);
    cmd.use_angle = 1;
    cmd.w = static_cast<float>((*targetLocation - robot->pos).angle());
    publishRobotCommand(cmd);

    return bt::Node::Status::Running;
}

void CoachDefend::onTerminate(bt::Node::Status s) {
    coach::DefensiveCoach::removeDefender(robot->id);
}
}
}