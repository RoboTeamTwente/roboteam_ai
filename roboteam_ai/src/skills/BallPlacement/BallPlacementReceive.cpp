//
// Created by mrlukasbos on 1-5-19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "BallPlacementReceive.h"

namespace rtt {
namespace ai {

BallPlacementReceive::BallPlacementReceive(string name, bt::Blackboard::Ptr blackboard)
: Receive(std::move(name), std::move(blackboard)) {}

bt::Node::Status BallPlacementReceive::onUpdate() {

    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) {
        return Status::Failure;
    }

    if (world::world->robotHasBall(robot->id, true)) {
        return Status::Success;
    }

    if (ball->pos.dist(coach::g_ballPlacement.getBallPlacementPos()) < 0.5) {
        publishRobotCommand();
        return Status::Success;
    }

    if (coach::g_pass.isPassed()) {
        // Check if the ball was deflected
//        if (passFailed()) {
//            publishRobotCommand(); // halt
//            return Status::Running;
//        }

        intercept();
    } else {

        Vector2 ballPlacementTarget = coach::g_ballPlacement.getBallPlacementPos();
        auto behindTargetPos = control::PositionUtils::getPositionBehindPositionToPosition(
                Constants::ROBOT_RADIUS(),
                ballPlacementTarget,
                ball->pos);

        moveToCatchPosition(behindTargetPos);
        if (isInPosition(behindTargetPos)) {
            coach::g_pass.setReadyToReceivePass(true);
        }
    }
    publishRobotCommand();
    return Status::Running;
}


void BallPlacementReceive::moveToCatchPosition(Vector2 position) {
    control::PosVelAngle pva = robot->getNumtreeGtp()->getPosVelAngle(robot, position);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);
    if (position.dist(robot->pos) < 0.6) {
        command.w = static_cast<float>((Vector2(ball->pos) - robot->pos).angle());
    } else {
        command.w = static_cast<float>((position - robot->pos).angle());
    }
}

// check if the robot is in the desired position to catch the ball
bool BallPlacementReceive::isInPosition(const Vector2& behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->id, true, ball->pos, 0.3*M_PI);
    bool isBehindTargetPos = behindTargetPos.dist(robot->pos) < 0.03;
    return isBehindTargetPos  && isAimedAtBall;
}

}
}