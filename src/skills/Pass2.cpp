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
#include "skills/Pass2.h"

namespace rtt {
namespace ai {

Pass2::Pass2(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Pass2::onInitialize() {
    robotToPassToID = - 1;
    passInitialized = false;
    hasShot = false;
    forcePass = false;
    fails = 0;
    maxTries = - 1;

}

Pass2::Status Pass2::onUpdate() {
    bool closeToBall = (robot->pos - ball->getPos()).length() < CLOSE_ENOUGH_TO_BALL;

    if (!passInitialized) {
        passInitialized = true;
    }

    robotToPassToID = properties->getInt("PassTo");
    if (robotToPassToID == -1) {
        return Status::Failure;
    }

    robotToPassTo = world::world->getRobotForId(robotToPassToID, true);

    if (didShootProperly()) {
        return Status::Success;
    }

    makeCommand();

    if ((command.kicker() == true || command.chipper() == true) && !hasShot) {
        hasShot = true;
    }


    publishRobotCommand();
    return Status::Running;
}

void Pass2::makeCommand() {
    std::cout << "made command in pass!" << std::endl;
    RobotCommand shotdata;

    shotdata = robot->getShotController()->getRobotCommand(*robot, getKicker(), forcePass, control::PASS,
                                                           false, control::HIGH);
    command = shotdata.makeROSCommand();
}

void Pass2::onTerminate(Status s) {
    hasShot = false;
    passInitialized = false;
    if (! coach::g_pass.isPassed() || forcePass) {
        coach::g_pass.resetPass(robot->id);
    }
    else if (s == Status::Success) {
    }
}

Vector2 Pass2::getKicker() {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return robotToPassTo->pos + distanceToKicker.rotate(robotToPassTo->angle);
}

void Pass2::initiatePass() {
    coach::g_pass.initiatePass(robot->id);
}

bool Pass2::didShootProperly() {
    bool ballIsMovingFast = Vector2(world::world->getBall()->getVel()).length() > 0.6;
    bool ballIsMovingToReceiver = true; //control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel,
                                        //robotToPassTo->pos, SUCCESSFUL_PASS_ANGLE);

    std::cout << "shot properly:" << (hasShot && ballIsMovingFast && ballIsMovingToReceiver) << std::endl;
    return (hasShot && ballIsMovingFast && ballIsMovingToReceiver);
}

Pass2::PassType Pass2::stringToType(const std::string& type) {
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

