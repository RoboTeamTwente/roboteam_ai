//
// Created by robzelluf on 1/22/19.
//

#include <coach/BallplacementCoach.h>
#include <control/PositionUtils.h>
#include <control/ControlUtils.h>
#include <utilities/Constants.h>
#include <control/ballHandling/BallHandlePosControl.h>
#include <control/BasicPosControl.h>
#include <interface/api/Input.h>
#include <world/Robot.h>
#include <world/WorldData.h>
#include "skills/Pass.h"

namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Pass::onInitialize() {
    coach::g_pass.resetPass(-1);

    if(properties->hasString("passType")) {
        passType = stringToType(properties->getString("passType"));
    } else {
        passType = DEFAULT;
    }

    robotToPassToID = - 1;
    passInitialized = false;
    hasShot = false;
    forcePass = false;
    fails = 0;
    if (properties->hasInt("failsUntilChip")) {
        maxTries = properties->getInt("failsUntilChip");
    }
    else {
        maxTries = - 1;
    }
}

Pass::Status Pass::onUpdate() {
    bool closeToBall = (robot->pos - ball->pos).length() < CLOSE_ENOUGH_TO_BALL;

    // Only do this if not close to the ball and the pass is not yet initialized
    if (! closeToBall && ! passInitialized) {
        RobotCommand robotCommand;

        robotToPassToID = coach::g_pass.getRobotBeingPassedTo();
        robotToPassTo = world::world->getRobotForId(robotToPassToID, true);

        if (! robotToPassTo || robotToPassToID == - 1) {
            robotCommand = robot->getNumtreePosControl()->getRobotCommand(robot, ball->pos);
        }
        else {
            robotCommand = robot->getBallHandlePosControl()->getRobotCommand(
                    robot, robotToPassTo->pos, control::BallHandlePosControl::TravelStrategy::FORWARDS);
        }

        command.mutable_vel()->set_x(robotCommand.vel.x);
        command.mutable_vel()->set_y(robotCommand.vel.y);
        command.set_w(robotCommand.angle);
        publishRobotCommand();
        return Status::Running;

    }
    else {
        if (! passInitialized) {
            passInitialized = true;
            initiatePass();
        }

        robotToPassToID = coach::g_pass.getRobotBeingPassedTo();
        if (robotToPassToID == - 1) {
            return Status::Failure;
        }

        robotToPassTo = world::world->getRobotForId(robotToPassToID, true);

        if (! coach::g_pass.validReceiver(robot, robotToPassTo)) {
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
        if (! forcePass && ! hasShot
                && ! control::ControlUtils::clearLine(ball->pos, robotToPassTo->pos, world::world->getWorld(), 1)) {

            // If the passType is defensive, force to immediately chip as soon as the pass is blocked
            if (passType == DEFENSIVE || passType == FREEKICK) {
                forcePass = true;
            } else

            if (maxTries == - 1) {
                return Status::Failure;
            }
            else {
                fails ++;
                if (fails >= maxTries) {
                    forcePass = true;
                }
                else {
                    coach::g_pass.resetPass(robot->id);
                    initiatePass();
                }
            }
        }

        makeCommand();

        if ((command.kicker() == true || command.chipper() == true) && ! hasShot) {
            hasShot = true;
        }
    }

    publishRobotCommand();
    return Status::Running;
}

void Pass::makeCommand() {
    RobotCommand shotdata;

    shotdata = robot->getShotController()->getRobotCommand(*robot, getKicker(), forcePass, control::PASS,
                                                           false, control::HIGH);
    command = shotdata.makeROSCommand();
}

void Pass::onTerminate(Status s) {
    hasShot = false;
    passInitialized = false;
    if (! coach::g_pass.isPassed() || forcePass) {
        coach::g_pass.resetPass(robot->id);
    }
    else if (s == Status::Success) {
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
    bool ballIsMovingFast = Vector2(world::world->getBall()->vel).length() > 0.6;
    bool ballIsMovingToReceiver = true;//control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel,
            //robotToPassTo->pos, SUCCESSFUL_PASS_ANGLE);

    return (hasShot && ballIsMovingFast && ballIsMovingToReceiver);
}

Pass::PassType Pass::stringToType(const std::string& type) {
    if (type == "defensive") {
        return DEFENSIVE;
    } else
    if (type == "freeKick") {
        return FREEKICK;
    } else {
        return DEFAULT;
    }
}

} // ai
} // rtt

