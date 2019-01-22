#include "Defend.h"
#include "../utilities/Coach.h"
#include "../control/ControlUtils.h"

namespace rtt {
namespace ai {

Defend::Defend(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Defend::onInitialize() {
    coach::Coach::addDefender(robot->id);
}


bt::Node::Status Defend::onUpdate() {
    control::ControlGoToPos gtp;
    Vector2 targetLocation = coach::Coach::getDefensivePosition(robot->id);
    auto velocities = gtp.goToPos(robot, targetLocation, control::GoToType::luTh);

    roboteam_msgs::RobotCommand cmd;
    cmd.id = robot->id;
    cmd.x_vel = static_cast<float>(velocities.x);
    cmd.y_vel = static_cast<float>(velocities.y);
    cmd.use_angle = 1;
    cmd.w = static_cast<float>((targetLocation - robot->pos).angle());
    publishRobotCommand(cmd);

    return bt::Node::Status::Running;
}

void Defend::onTerminate(bt::Node::Status s) {
    coach::Coach::removeDefender(robot->id);
}

} // ai
} // rtt