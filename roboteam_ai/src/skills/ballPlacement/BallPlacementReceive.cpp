//
// Created by mrlukasbos on 1-5-19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "BallPlacementReceive.h"
#include "roboteam_ai/src/control/numTrees/NumTreePosControl.h"

namespace rtt {
namespace ai {

BallPlacementReceive::BallPlacementReceive(string name, bt::Blackboard::Ptr blackboard)
: Receive(std::move(name), std::move(blackboard)) {}

bt::Node::Status BallPlacementReceive::onUpdate() {

    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) {
        return Status::Failure;
    }

    if (coach::g_pass.isPassed() && ball->vel.length() <= Constants::BALL_STILL_VEL()) {
        return Status::Success;
    }

    if ((ball->pos - coach::g_ballPlacement.getBallPlacementPos()).length() < 0.25) {
        return Status::Success;
    }

    if (coach::g_pass.isPassed()) {
//        if(ballDeflected()) {
//            return Status::Failure;
//        }
        command.dribbler = 31;
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


void BallPlacementReceive::moveToCatchPosition(const Vector2& position) {
    auto robotCommand = robot->getNumtreePosControl()->getRobotCommand(robot, position);
    command.x_vel = static_cast<float>(robotCommand.vel.x);
    command.y_vel = static_cast<float>(robotCommand.vel.y);
    if (position.dist(robot->pos) < 0.6) {
        command.w = static_cast<float>((Vector2(ball->pos) - robot->pos).angle());
    } else {
        command.w = static_cast<float>((position - robot->pos).angle());
    }
}

// check if the robot is in the desired position to catch the ball
bool BallPlacementReceive::isInPosition(const Vector2& behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->id, true, ball->pos, 0.3*M_PI);
    bool isBehindTargetPos = behindTargetPos.dist(robot->pos) < 0.10;
    return isBehindTargetPos  && isAimedAtBall;
}

}
}