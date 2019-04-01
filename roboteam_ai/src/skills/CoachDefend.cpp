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
gtp.setCanMoveInDefenseArea(true);
}


bt::Node::Status CoachDefend::onUpdate() {
    coach::DefensiveCoach::updateDefenderLocations();
    auto targetLocation = coach::DefensiveCoach::getDefenderPosition(robot->id);
    if (!targetLocation){
        std::cerr<<"Could not find the location of defender "<< robot->id<< " in calculated positions!"<<std::endl;
        return bt::Node::Status::Running;
    }

    auto velocities = gtp.getPosVelAngle(robot, targetLocation->first);
    if ((targetLocation->first-robot->pos).length()<0.02){
        command.x_vel = 0;
        command.y_vel = 0;
        command.w = static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second));
    }
    else if ((targetLocation->first-robot->pos).length()<0.12){
        command.x_vel = static_cast<float>(velocities.vel.x);
        command.y_vel = static_cast<float>(velocities.vel.y);
        command.w = velocities.angle;
    }
    else{
        command.x_vel = static_cast<float>(velocities.vel.x);
        command.y_vel = static_cast<float>(velocities.vel.y);
        command.w = velocities.angle;
    }
    publishRobotCommand();

    return bt::Node::Status::Running;
}

void CoachDefend::onTerminate(bt::Node::Status s) {
    coach::DefensiveCoach::removeDefender(robot->id);
}
}
}