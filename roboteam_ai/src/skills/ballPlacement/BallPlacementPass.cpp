//
// Created by mrlukasbos on 3-5-19.
//

#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "BallPlacementPass.h"

namespace rtt {
namespace ai {


BallPlacementPass::BallPlacementPass(string name, bt::Blackboard::Ptr blackboard)
    : Pass(name, blackboard) { }


void BallPlacementPass::onInitialize() {
    robotToPassToID = -1;
    hasShot = false;
}

bt::Node::Status BallPlacementPass::onUpdate() {

    targetPos = coach::g_ballPlacement.getBallPlacementPos();


    robotToPassToID = coach::g_pass.getRobotBeingPassedTo();
    if (robotToPassToID == -1) {
        std::cout << "the robot to pass to id is -1" << std::endl;
        return Status::Failure;
    }

    robotToPassTo = world::world->getRobotForId(robotToPassToID, true);

    if ((ball->pos - targetPos).length() < 0.5) {
        return Status::Success;
    }

    if (didShootProperly()) {
        coach::g_pass.setPassed(true);
        return Status::Success;
    }

    /*
     * Make the shot if the receiver is ready
     * Otherwise we can already drive to the position but wait while close
     * When receiver is ready we can shoot
     */
    if (!coach::g_pass.isPassed()) {
        if (coach::g_pass.isReadyToReceivePass()) {
            robot->getShotController()->makeCommand(robot->getShotController()->getShotData(*robot, getKicker(), false, control::BallSpeed::PASS, false, control::ShotPrecision::MEDIUM), command);
            if(command.kicker == true && !hasShot) {
                hasShot = true;
            }
        } else if (robot->pos.dist(ball->pos) > 0.5) {
            auto pva = robot->getNumtreePosControl()->getPosVelAngle(robot, ball->pos);
            command.x_vel = pva.vel.x;
            command.y_vel = pva.vel.y;
            command.w = pva.angle;
        } else {
            command.w = (ball->pos - robot->pos).angle();
        }
    }


    publishRobotCommand();
    return Status::Running;
}


}
}