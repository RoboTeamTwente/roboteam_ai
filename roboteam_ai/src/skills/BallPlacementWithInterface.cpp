//
// Created by thijs on 15-5-19.
//

#include "BallPlacementWithInterface.h"

//
// Created by baris on 10-5-19.
//

#include <roboteam_ai/src/interface/api/Output.h>
#include "DriveWithInterface.h"

namespace rtt {
namespace ai {
BallPlacementWithInterface::BallPlacementWithInterface(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}

Skill::Status BallPlacementWithInterface::onUpdate() {

    if (interface::Output::usesRefereeCommands()) {
        return Status::Failure;
    }

    Vector2 targetPos = interface::Output::getInterfaceMarkerPosition();

//    if ((targetPos - ball->pos).length() < 0.15) {
//        if (robot->getDribblerState() == 0 && robot->isDribblerReady()) {
//            // The ball is stopped here.
//            // Then we should drive backwards
//
//            targetPos = (robot->pos - ball->pos).stretchToLength(0.3);
//
//            auto pva = basicPosControl.getPosVelAngle(robot, targetPos);
//            Vector2 vel = control::ControlUtils::velocityLimiter(pva.vel, 1.0, 0.3);
//            command.x_vel = vel.x;
//            command.y_vel = vel.y;
//            command.w = vel.toAngle() + M_PI;
//            publishRobotCommand();
//            return Status::Running;
//
//
//        }
//    }
    Angle targetAngle = robot->angle;
    if (lockedAngle != 0.0) {
        targetAngle = lockedAngle;
    }
    else if ((targetPos - ball->pos).length() < 0.1) {
        lockedAngle = robot->angle;
    }

    auto rc = ballHandlePosControl.getRobotCommand(robot, targetPos, targetAngle);
    Vector2 velocity = control::ControlUtils::velocityLimiter(rc.vel, Constants::MAX_VEL());
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = rc.angle;
    command.dribbler = rc.dribbler;
    publishRobotCommand();
    return Status::Running;
}

}
}