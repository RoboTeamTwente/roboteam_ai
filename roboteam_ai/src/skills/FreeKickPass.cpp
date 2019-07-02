//
// Created by robzelluf on 7/2/19.
//

#include "FreeKickPass.h"
#include <roboteam_ai/src/control/ballHandling/BallHandlePosControl.h>

namespace rtt {
namespace ai {

FreeKickPass::FreeKickPass(string name, bt::Blackboard::Ptr blackboard)
        :Pass(std::move(name), std::move(blackboard)) { }

void FreeKickPass::onInitialize() {
    if(properties->hasInt("maxTries")) {
        maxTries = properties->getInt("maxTries");
    } else {
        maxTries = 3;
    }

    robotToPassToID = - 1;
    passInitialized = false;
    hasShot = false;
    fails = 0;
    forcePass = false;
}

FreeKickPass::Status FreeKickPass::onUpdate() {
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

        command.x_vel = robotCommand.vel.x;
        command.y_vel = robotCommand.vel.y;
        command.w = robotCommand.angle;
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
        if (! coach::g_pass.validReceiver(robot, robotToPassTo, true)) {
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

        auto shotdata = robot->getShotController()->getRobotCommand(*robot, getKicker(), false, control::BallSpeed::PASS,
                                                                    true, control::ShotPrecision::HIGH);
        command = shotdata.makeROSCommand();
        if ((command.kicker == true || command.chipper == true) && ! hasShot) {
            hasShot = true;
        }
    }

    publishRobotCommand();
    return Status::Running;
}

}
}
