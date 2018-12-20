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

        if (newRandom && newPos) {
            const roboteam_msgs::GeometryFieldSize &field = Field::get_field();
            const double &length = field.field_length;
            const double &width = field.field_width;
            int randomX = std::rand();
            int randomY = std::rand();
            targetPos = {randomX*2.32830644e-10*length*2 - length*0.5, randomY*2.32830644e-10*width*2 - width*0.5};

            newPos = false;
        }
        else if (! newRandom && newPos) {
            auto ball = World::getBall();
            targetPos = ball.pos;
        }

    goToPos.goToPos(robot, targetPos, goToType::luTh);

    deltaPos = targetPos - (Vector2)robot->pos;
    if (abs(deltaPos.length()) < 0.5 && --counter < 1) {
        newPos = true;
        newRandom = !newRandom;
        counter = 100;
        goToPos.clear(goToType::luTh);
    }

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