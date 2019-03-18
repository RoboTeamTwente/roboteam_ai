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
    auto form = properties->getString("Formation");
    setFormation(form);
}

bt::Node::Status EnterFormation::onUpdate() {
    switch (formation) {

        case Normal:{
            auto robotPos = rtt::Vector2(robot->pos);
            Vector2 targetLocation = coach::Coach::getFormationPosition(robot->id);
            Vector2 targetToLookAtLocation = Field::get_their_goal_center();

            roboteam_msgs::RobotCommand cmd;
            cmd.id = robot->id;
            cmd.use_angle = 1;

            if (robotPos.dist(targetLocation) > Constants::NUMTREE_ERROR_MARGIN()) {
                auto velocities = gtp.goToPos(robot, targetLocation, control::PosControlType::NUMERIC_TREES);
                cmd.x_vel = static_cast<float>(velocities.vel.x);
                cmd.y_vel = static_cast<float>(velocities.vel.y);
                cmd.w = static_cast<float>((targetLocation-robot->pos).angle());
            } else { // we are at the right location
                cmd.w = static_cast<float>((targetToLookAtLocation-robot->pos).angle());
            }
            publishRobotCommand(cmd);
            return bt::Node::Status::Running;
        }
        case Penalty:{

        }
        case FreeKick:{

        }
    }
    return bt::Node::Status::Failure;


}

void EnterFormation::onTerminate(bt::Node::Status s) {
    coach::Coach::removeFormationRobot(robot->id);
}
void EnterFormation::setFormation(std::string property) {
    if (property == "Penalty") {
        formation = Penalty;
    }
    else if (property == "FreeKick") {
        formation = FreeKick;
    }
    else { // this also means that if you dont have this set in the tree it will be normal
        formation = Normal;
    }

}

} // ai
} // rtt