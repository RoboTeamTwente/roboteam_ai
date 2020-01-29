//
// Created by mrlukasbos on 1-5-19.
//

#include "skills/ballPlacement/BallPlacementReceive.h"
#include <coach/BallplacementCoach.h>
#include <coach/PassCoach.h>
#include "control/ControlUtils.h"
#include "control/numTrees/NumTreePosControl.h"

namespace rtt::ai {

BallPlacementReceive::BallPlacementReceive(string name, bt::Blackboard::Ptr blackboard) : Receive(std::move(name), std::move(blackboard)) {}

bt::Node::Status BallPlacementReceive::onUpdate() {
    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) {
        return Status::Failure;
    }

    if (coach::g_pass.isPassed() && ball->getVel().length() <= Constants::BALL_STILL_VEL()) {
        return Status::Success;
    }

    if ((ball->getPos() - coach::g_ballPlacement.getBallPlacementPos()).length() < 0.25) {
        return Status::Success;
    }

    if (coach::g_pass.isPassed()) {
        //        if(ballDeflected()) {
        //            return Status::Failure;
        //        }
        command.set_dribbler(31);
        intercept();
    } else {
        Vector2 ballPlacementTarget = coach::g_ballPlacement.getBallPlacementPos();
        auto behindTargetPos = control::PositionUtils::getPositionBehindPositionToPosition(Constants::ROBOT_RADIUS(), ballPlacementTarget, ball->getPos());

        moveToCatchPosition(behindTargetPos);
        if (isInPosition(behindTargetPos)) {
            coach::g_pass.setReadyToReceivePass(true);
        }
    }
    publishRobotCommand();
    return Status::Running;
}

void BallPlacementReceive::moveToCatchPosition(const Vector2 &position) {
    auto robotCommand = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, position);
    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    if (position.dist(robot->pos) < 0.6) {
        command.set_w((Vector2(ball->getPos()) - robot->pos).angle());
    } else {
        command.set_w((position - robot->pos).angle());
    }
}

// check if the robot is in the desired position to catch the ball
bool BallPlacementReceive::isInPosition(const Vector2 &behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->id, true, ball->getPos(), 0.3 * M_PI);
    bool isBehindTargetPos = behindTargetPos.dist(robot->pos) < 0.10;
    return isBehindTargetPos && isAimedAtBall;
}

}  // namespace rtt::ai