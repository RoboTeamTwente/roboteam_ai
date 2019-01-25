//
// Created by mrlukasbos on 23-1-19.
//

#include "EnterFormation.h"
#include "../utilities/Coach.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

EnterFormation::EnterFormation(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void EnterFormation::onInitialize() {
    coach::Coach::addFormationRobot(robot->id);
}

bt::Node::Status EnterFormation::onUpdate() {
    auto robotPos = rtt::Vector2(robot->pos);
    Vector2 targetLocation = coach::Coach::getFormationPosition(robot->id);
    Vector2 targetToLookAtLocation = Field::get_their_goal_center();

    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.use_angle = 1;

    if (robotPos.dist(targetLocation) > constants::GOTOPOS_LUTH_ERROR_MARGIN) {
        auto velocities = gtp.goToPos(robot, targetLocation, control::GoToType::luTh);
        cmd.x_vel = velocities.x;
        cmd.y_vel = velocities.y;
        cmd.w = static_cast<float>((targetLocation-robot->pos).angle());
    } else { // we are at the right location
        cmd.w = static_cast<float>((targetToLookAtLocation-robot->pos).angle());

        // if we have the right angle
        if (robot->angle > cmd.w - 0.2 && robot->angle < cmd.w + 0.2) {
            return bt::Node::Status::Success;
        }
    }
    publishRobotCommand(cmd);
    return bt::Node::Status::Running;
}

void EnterFormation::onTerminate(bt::Node::Status s) {
    coach::Coach::removeFormationRobot(robot->id);
}

} // ai
} // rtt