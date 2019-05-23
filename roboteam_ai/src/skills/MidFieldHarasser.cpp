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
    robotBeingHarassed = -1;
}

Skill::Status MidFieldHarasser::onUpdate() {
    targetPos = getHarassTarget();

    auto newPosition = robot->getNumtreeGtp()->getPosVelAngle(robot, targetPos);
    Vector2 velocity = newPosition.vel;

    // If there is a robot being harassed, drive slower than it if too close
    if (robotBeingHarassed != - 1) {
        RobotPtr opponent = world::world->getRobotForId(robotBeingHarassed, false);
        if (opponent && ((opponent->pos - robot->pos).length() < HARASSING_SAFETY_MARGINS)) {
            double opponentVelocityLength = opponent->vel.length();
            velocity = control::ControlUtils::velocityLimiter(velocity, opponentVelocityLength*0.8,
                    Constants::MIN_VEL());
        }
        command.x_vel = static_cast<float>(velocity.x);
        command.y_vel = static_cast<float>(velocity.y);
        command.w =  static_cast<float>(getHarassAngle().getAngle());
        command.use_angle = 1;
        publishRobotCommand();

        return Status::Running;
    }
}

void MidFieldHarasser::onTerminate(Skill::Status s) {
    command.w = static_cast<float>(robot->angle);
    command.use_angle = 1;
    command.x_vel = 0;
    command.y_vel = 0;
    myIndex = - 1;

    publishRobotCommand();
}


Vector2 MidFieldHarasser::getHarassTarget() {
    auto harassTarget =  coach::g_harassRobotCoach.getHarassPosition(robot, myIndex);
    robotBeingHarassed = harassTarget.harassRobot;
    return harassTarget.harassPosition;
}

Angle MidFieldHarasser::getHarassAngle() {
    return coach::g_harassRobotCoach.getHarassAngle(robot, myIndex);
}

} //ai
} //rtt
