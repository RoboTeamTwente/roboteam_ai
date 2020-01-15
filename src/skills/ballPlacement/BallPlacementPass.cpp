//
// Created by mrlukasbos on 3-5-19.
//

#include <coach/BallplacementCoach.h>
#include "skills/ballPlacement/BallPlacementPass.h"

namespace rtt::ai {


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

    robotToPassTo = world->getRobotForId(robotToPassToID, true);

    if ((ball->getPos() - targetPos).length() < 0.5) {
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
            auto shotData = robot->getShotController()->getRobotCommand(*robot, getKicker(), false, control::BallSpeed::BALL_PLACEMENT, false, control::ShotPrecision::MEDIUM, 3);
            command = shotData.makeROSCommand();
            if(command.kicker() && !hasShot) {
                hasShot = true;
            }
        } else if (robot->pos.dist(ball->getPos()) > 0.5) {
            auto robotCommand = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, ball->getPos());
            command.mutable_vel()->set_x(robotCommand.vel.x);
            command.mutable_vel()->set_y(robotCommand.vel.y);
            command.set_w(robotCommand.angle);
        } else {
            command.set_w((ball->getPos() - robot->pos).angle());
        }
    }


    publishRobotCommand();
    return Status::Running;
}


}