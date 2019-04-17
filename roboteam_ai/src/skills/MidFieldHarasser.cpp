//
// Created by thijs on 10-4-19.
//

#include "MidFieldHarasser.h"
#include "../coach/midField/HarassRobotCoach.h"

namespace rtt {
namespace ai {

MidFieldHarasser::MidFieldHarasser(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) {
}

void MidFieldHarasser::onInitialize() {
    myIndex = -1;
}

Skill::Status MidFieldHarasser::onUpdate() {

    targetPos = getHarassPosition();

    auto newPosition = goToPos.getPosVelAngle(robot, targetPos);
    Vector2 velocity = newPosition.vel;
    velocity = control::ControlUtils::velocityLimiter(velocity);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    //command.w = static_cast<float>(newPosition.angle);
    command.w = static_cast<float>(getHarassAngle().getAngle());
    command.use_angle = 1;
    publishRobotCommand();

    return Status::Running;
}

void MidFieldHarasser::onTerminate(Skill::Status s) {
    command.w = static_cast<float>(robot->angle);
    command.use_angle = 1;
    command.x_vel = 0;
    command.y_vel = 0;
    myIndex = - 1;

    publishRobotCommand();
}


Vector2 MidFieldHarasser::getHarassPosition() {
    return coach::g_harassRobotCoach.getHarassPosition(robot, myIndex);
}

Angle MidFieldHarasser::getHarassAngle() {
    return coach::g_harassRobotCoach.getHarassAngle(robot, myIndex);
}

} //ai
} //rtt