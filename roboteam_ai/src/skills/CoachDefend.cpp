//
// Created by rolf on 5-3-19.
//

#include <roboteam_ai/src/world/Field.h>
#include "CoachDefend.h"
#include "roboteam_ai/src/coach/defence/DefenceDealer.h"
#include "roboteam_ai/src/control/ControlUtils.h"
namespace rtt {
namespace ai {
CoachDefend::CoachDefend(std::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void CoachDefend::onInitialize() {
  //  numTreeGtp.setCanMoveInDefenseArea(false);
}

bt::Node::Status CoachDefend::onUpdate() {
    coach::g_DefenceDealer.addDefender(robot->id);
    auto targetLocation = coach::g_DefenceDealer.getDefenderPosition(robot->id);
    if (! targetLocation) {
        command.x_vel = 0;
        command.y_vel = 0;
        command.w=0;
        publishRobotCommand();
        return bt::Node::Status::Running;
    }


    RobotCommand velocities;
    // choosing numtrees or other
    if (useBasicGtp(targetLocation->first)){
        velocities=robot->getBasicPosControl()->getRobotCommand(robot,targetLocation->first);
    }
    else{
         velocities=robot->getNumtreePosControl()->getRobotCommand(robot, targetLocation->first);
    }
    if ((targetLocation->first - robot->pos).length() < Constants::ROBOT_RADIUS()) {
        command.x_vel = 0;
        command.y_vel = 0;
        command.w = static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second));
    }
    else {
        command.x_vel = static_cast<float>(velocities.vel.x);
        command.y_vel = static_cast<float>(velocities.vel.y);
        if ((targetLocation->first - robot->pos).length() < Constants::ROBOT_RADIUS()) {
            command.w = static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second));
        }
        else {
            command.w = velocities.angle;
        }
    }
    publishRobotCommand();
    return bt::Node::Status::Running;
}
bool CoachDefend::useBasicGtp(Vector2 targetLocation) {
    LineSegment driveLine(robot->pos,targetLocation);
    if (driveLine.length()>=0.2){
        return false;
    }
    // if line intersects our defence area or other robots we do not do it.
    if (world::field->getDefenseArea(true,0.15,false).doesIntersect(driveLine)){
        return false;
    }
    auto robots=world::world->getThem();
    for (const auto& worldRobot: robots){
        if (worldRobot->id != robot->id){
            if (driveLine.distanceToLine(worldRobot->pos)<Constants::ROBOT_RADIUS()){
                return false;
            }
        }
    }
    return true;
}
}
}