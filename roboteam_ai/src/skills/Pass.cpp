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
        shotControl = std::make_shared<control::ShotController>(control::ShotPrecision::HIGH, control::BallSpeed::PASS, false);
    } else {
        shotControl = std::make_shared<control::ShotController>(control::ShotPrecision::MEDIUM, control::BallSpeed::PASS, false);
    }

    passInitialized = false;
    shot = false;
}

Pass::Status Pass::onUpdate() {
    Vector2 target = world::field->get_their_goal_center();

    bool closeToBall = robot->pos.dist(ball->pos) < CLOSE_ENOUGH_TO_BALL;

    if(!closeToBall && !passInitialized) {
        auto pva = numTreeGtp.getPosVelAngle(robot, ball->pos);
        pva.vel = control::ControlUtils::velocityLimiter(pva.vel);
        command.x_vel = pva.vel.x;
        command.y_vel = pva.vel.y;
        command.w = (target - robot->pos).toAngle();
    } else {
        if(!passInitialized) {
            passInitialized = true;
            initiatePass();
        }

        robotToPassToID = coach::g_pass.getRobotBeingPassedTo();

        if (robotToPassToID == -1) {
            std::cout << "Receiver id -1" << std::endl;
            return Status::Failure;
        }

        robotToPassTo = world::world->getRobotForId(robotToPassToID, true);
        if (!coach::g_pass.checkIfValidReceiver(robot->id, robotToPassToID)) {
            std::cout << "No valid receiver anymore!" << std::endl;
            return Status::Failure;
        }

        target = getKicker();

        if (!shot && !control::ControlUtils::clearLine(ball->pos, target, world::world->getWorld(), 1)) {
            std::cerr << "Passline not clear anymore" << std::endl;
            return Status::Failure;
        }

        bool ballIsMovingFast = Vector2(world::world->getBall()->vel).length() > 0.8;
        bool ballIsShotTowardsReceiver = control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel, getKicker());

        if (ballIsMovingFast && ballIsShotTowardsReceiver) {
            coach::g_pass.setPassed(true);
            return Status::Success;
        }

        shotControl->makeCommand(shotControl->getShotData(*robot, target), command);
        if (command.kicker) {
            shot = true;
        }
    }

    publishRobotCommand();

    return Status::Running;

}

void Pass::onTerminate(Status s) {
    if (!coach::g_pass.isPassed()) {
        coach::g_pass.resetPass();
    }
}

Vector2 Pass::getKicker() {
    Vector2 distanceToKicker = {Constants::CENTRE_TO_FRONT(), 0};
    return robotToPassTo->pos + distanceToKicker.rotate(robotToPassTo->angle);
}

void Pass::initiatePass() {
    robotToPassToID = ballPlacement ? coach::g_pass.getRobotBeingPassedTo() : coach::g_pass.initiatePass(robot->id);
}


} // ai
} // rtt

