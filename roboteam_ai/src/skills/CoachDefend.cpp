//
// Created by rolf on 5-3-19.
//

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
    auto velocities = robot->getNumtreePosControl()->getPosVelAngle(robot, targetLocation->first);
    if ((targetLocation->first - robot->pos).length() < 0.02) {
        command.x_vel = 0;
        command.y_vel = 0;
        command.w = static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second));
    }
    else {
        command.x_vel = static_cast<float>(velocities.vel.x);
        command.y_vel = static_cast<float>(velocities.vel.y);
        if ((targetLocation->first - robot->pos).length() < 2*Constants::ROBOT_RADIUS()) {
            command.w = static_cast<float>(control::ControlUtils::constrainAngle(targetLocation->second));
        }
        else {
            command.w = velocities.angle;
        }
    }
    publishRobotCommand();
    return bt::Node::Status::Running;
}

}
}