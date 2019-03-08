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
    coach::DefensiveCoach::updateDefenderLocations();
    auto targetLocation = coach::DefensiveCoach::getDefenderPosition(robot->id);
    if (!targetLocation){
        std::cerr<<"Could not find the location of defender "<< robot->id<< " in calculated positions!"<<std::endl;
        return bt::Node::Status::Running;
    }
    auto velocities = gtp.goToPos(robot, targetLocation->first, control::GoToType::basic);
    velocities=control::ControlUtils::VelocityLimiter(velocities);
    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.use_angle = 1;
    if ((targetLocation->first-robot->pos).length()<0.02){
        cmd.x_vel = 0;
        cmd.y_vel = 0;
        cmd.w = static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second));
    }
    else if ((targetLocation->first-robot->pos).length()<0.12){
        cmd.x_vel = static_cast<float>(velocities.x);
        cmd.y_vel = static_cast<float>(velocities.y);
        cmd.w = static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second));
    }
    else{
        cmd.x_vel = static_cast<float>(velocities.x);
        cmd.y_vel = static_cast<float>(velocities.y);
        cmd.w = static_cast<float>((targetLocation->first - robot->pos).angle());
    }

    publishRobotCommand(cmd);

    return bt::Node::Status::Running;
}

void CoachDefend::onTerminate(bt::Node::Status s) {
    coach::DefensiveCoach::removeDefender(robot->id);
}
}
}