//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/interface/drawer.h>
#include "Pass.h"

namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) { }

void Pass::onInitialize() {
    ballPlacement = properties->getBool("BallPlacement");
    robotToPassToID = -1;
    if (ballPlacement) {
        shotControl = std::make_shared<control::ShotController>(control::ShotPrecision::HIGH, control::BallSpeed::PASS, true);
    } else {
        shotControl = std::make_shared<control::ShotController>(control::ShotPrecision::MEDIUM, control::BallSpeed::PASS, true);
    }

    passInitialized = false;
    shot = false;
}

Pass::Status Pass::onUpdate() {
    bool closeToBall = (robot->pos - ball->pos).length() < CLOSE_ENOUGH_TO_BALL;

    if(!closeToBall && !passInitialized) {
        auto pva = numTreeGtp.getPosVelAngle(robot, ball->pos);
        command.x_vel = pva.vel.x;
        command.y_vel = pva.vel.y;
        command.w = pva.angle;
        publishRobotCommand();
        return Status::Running;

    } else {
        if(!passInitialized) {
            passInitialized = true;
            initiatePass();
        }

        robotToPassToID = coach::g_pass.getRobotBeingPassedTo();
        if (robotToPassToID == -1) {
            return Status::Failure;
        }

        robotToPassTo = world::world->getRobotForId(robotToPassToID, true);
        if(!coach::g_pass.validReceiver(robot, robotToPassTo)) {
            return Status::Failure;
        }

        bool ballIsMovingFast = Vector2(world::world->getBall()->vel).length() > 0.8;
        bool ballIsMovingToReceiver = control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel, robotToPassTo->pos);

        if (shot && ballIsMovingFast && ballIsMovingToReceiver) {
            coach::g_pass.setPassed(true);
            return Status::Success;
        }

        if(!shot && !control::ControlUtils::clearLine(ball->pos, robotToPassTo->pos, world::world->getWorld(), 1)) {
            return Status::Failure;
        }

        shotControl->makeCommand(shotControl->getShotData(* robot, getKicker()), command);
        if (command.kicker == true && !shot) {
            shot = true;
        }
    }

    publishRobotCommand();
    return Status::Running;
}

void Pass::onTerminate(Status s) {
    shot = false;
    passInitialized = false;
    if (!coach::g_pass.isPassed()) {
        coach::g_pass.resetPass(robot->id);
    } else if (s == Status::Success) {
    }
}

Vector2 Pass::getKicker() {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return robotToPassTo->pos + distanceToKicker.rotate(robotToPassTo->angle);
}

void Pass::initiatePass() {
     if (ballPlacement) {
         coach::g_pass.getRobotBeingPassedTo();
     } else {
         coach::g_pass.initiatePass(robot->id);
     }
}


} // ai
} // rtt

