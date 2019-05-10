//
// Created by baris on 10-5-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include "DriveWithInterface.h"
namespace rtt {
namespace ai {
DriveWithInterface::DriveWithInterface(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}
Skill::Status DriveWithInterface::onUpdate() {

    Vector2 targetPos = interface::Output::getBallPlacementTarget();

    Vector2 velocity = goToPos->getPosVelAngle(robot, targetPos).vel;
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = static_cast<float>((targetPos - robot->pos).angle());
    publishRobotCommand();
    return Status::Running;
}
void DriveWithInterface::onInitialize() {

}
void DriveWithInterface::onTerminate(Skill::Status s) {
    Skill::onTerminate(s);
}
}
}