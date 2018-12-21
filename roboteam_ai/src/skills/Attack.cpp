//
// Created by thijs on 17-12-18.
//

#include "Attack.h"


namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Init the GoToPos skill
void Attack::onInitialize() {
    robot = getRobotFromProperties(properties);
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    updateRobot();
    if (! robot) return Status::Running;



    goToPos.goToPos(robot, targetPos, GoToType::luTh);


    return Status::Running;
}

void Attack::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.w = static_cast<float>(deltaPos.angle());

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

} // ai
} // rtt