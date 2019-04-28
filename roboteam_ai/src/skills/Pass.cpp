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
}

Pass::Status Pass::onUpdate() {
    Vector2 target = world::field->get_their_goal_center();

    bool closeToBall = robot->pos.dist(ball->pos) < CLOSE_ENOUGH_TO_BALL;

    if (robotToPassToID != -1) {
        robotToPassTo = world::world->getRobotForId(robotToPassToID, true);
        if (robotToPassTo) {
            target = getKicker();

            bool ballIsMovingFast = Vector2(world::world->getBall()->vel).length() > 0.8;
            bool ballIsShotTowardsReceiver = control::ControlUtils::objectVelocityAimedToPoint(ball->pos, ball->vel, getKicker());

            if (ballIsMovingFast && ballIsShotTowardsReceiver) {
                coach::g_pass.setPassed(true);
                return Status::Success;
            }
        }
    } else if (closeToBall || ballPlacement) {
        initiatePass();
    }

    control::ShotData shotData = shotControl->getShotData(*robot, target);
    command.x_vel = shotData.vel.x;
    command.y_vel = shotData.vel.y;
    command.w = shotData.angle.getAngle();
    command.kicker = shotData.kick;
    command.kicker_forced = shotData.kick;
    command.kicker_vel = shotData.kickSpeed;
    command.geneva_state = shotData.genevaState;
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

