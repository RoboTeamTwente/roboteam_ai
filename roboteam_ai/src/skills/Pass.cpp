//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include <roboteam_ai/src/control/PositionUtils.h>
#include <roboteam_ai/src/utilities/Constants.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/interface/api/Input.h>
#include "Pass.h"

namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) { }

void Pass::onInitialize() {
    robotToPassToID = -1;
    passInitialized = false;
    hasShot = false;
    chip = false;
    fails = 0;
    if (properties->hasInt("failsUntilChip")) {
        failsUntilChip = properties->getInt("failsUntilChip");
    } else {
        failsUntilChip = -1;
    }
}

Pass::Status Pass::onUpdate() {
    bool closeToBall = (robot->pos - ball->pos).length() < CLOSE_ENOUGH_TO_BALL;

    if(!closeToBall && !passInitialized) {
        auto pva = robot->getNumtreeGtp()->getPosVelAngle(robot, ball->pos);
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

         if (didShootProperly()) {
             coach::g_pass.setPassed(true);
             return Status::Success;
         }

        ///Check if:
        // Not already decided to chip
        // Not having already tried a shot
        // If this is both not the case, check if there's a clear line to the target
        // If not, either ++ fails or fail immediately
        if(!chip && !hasShot && !control::ControlUtils::clearLine(ball->pos, robotToPassTo->pos, world::world->getWorld(), 1)) {
            if (failsUntilChip == -1) {
                return Status::Failure;
            } else {
                fails++;
                if (fails >= failsUntilChip) {
                    chip = true;
                } else {
                    coach::g_pass.resetPass(robot->id);
                    initiatePass();
                }
            }
        }

        robot->getShotController()->makeCommand(robot->getShotController()->getShotData(* robot, getKicker(), chip), command);
        if ((command.kicker == true || command.chipper == true) && !hasShot) {
            hasShot = true;
        }
    }

    publishRobotCommand();
    return Status::Running;
}

void Pass::onTerminate(Status s) {
    hasShot = false;
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
    coach::g_pass.initiatePass(robot->id);
}

bool Pass::didShootProperly() {
    bool ballIsMovingFast = Vector2(world::world->getBall()->vel).length() > 0.8;
    bool ballIsMovingToReceiver = control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel, robotToPassTo->pos);

    return (hasShot && ballIsMovingFast && ballIsMovingToReceiver);
}


} // ai
} // rtt

