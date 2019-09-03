//
// Created by thijs on 10-4-19.
//

#include "skills/MidFieldHarasser.h"

namespace rtt {
namespace ai {

MidFieldHarasser::MidFieldHarasser(string name, bt::Blackboard::Ptr blackboard)
        : Skill(std::move(name), std::move(blackboard)) {
}

void MidFieldHarasser::onInitialize() {
    robotBeingHarassed = -1;
    coach::g_midFieldCoach.addMidFielder(robot);
}

Skill::Status MidFieldHarasser::onUpdate() {
    targetPos = getHarassTarget();

    auto newPosition = robot->getNumtreePosControl()->getRobotCommand(robot, targetPos);
    Vector2 velocity = newPosition.vel;

    // If there is a robot being harassed, drive slower than it if too close
    if (robotBeingHarassed != - 1) {
        RobotPtr opponent = world::world->getRobotForId(robotBeingHarassed, false);
        if (opponent && ((opponent->pos - robot->pos).length() < HARASSING_SAFETY_MARGINS)) {
            double opponentVelocityLength = opponent->vel.length();

            //TODO: Base this on the actual rules
            if (opponentVelocityLength > 2.0) {
                velocity = control::ControlUtils::velocityLimiter(velocity, opponentVelocityLength * 0.8,
                                                                  Constants::MIN_VEL());
            }
            command.set_w((opponent->pos - robot->pos).angle());
        }
    } else {
        command.set_w(newPosition.angle);
    }

    command.mutable_vel()->set_x(static_cast<float>(velocity.x));
    command.mutable_vel()->set_y(static_cast<float>(velocity.y));
    command.set_use_angle(true);
    publishRobotCommand();

    return Status::Running;
}

void MidFieldHarasser::onTerminate(Skill::Status s) {
    coach::g_midFieldCoach.removeMidFielder(robot);
}


Vector2 MidFieldHarasser::getHarassTarget() {
    auto harassTarget = coach::g_midFieldCoach.getTargetPosition(robot);

    if (!world::field->pointIsInField(harassTarget.targetPosition, 0.20)) {
        harassTarget.targetPosition = robot->pos;
    }

    interface::Input::drawData(interface::Visual::PATHFINDING, {harassTarget.targetPosition}, Qt::darkYellow, robot->id,
                               interface::Drawing::CROSSES, 3, 3);

    robotBeingHarassed = harassTarget.targetRobot;
    return harassTarget.targetPosition;
}

} //ai
} //rtt
