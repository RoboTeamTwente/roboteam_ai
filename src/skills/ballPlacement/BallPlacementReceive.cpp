//
// Created by mrlukasbos on 1-5-19.
//

#include "coach/BallplacementCoach.h"
#include "control/PositionUtils.h"
#include "control/ControlUtils.h"
#include "skills/ballPlacement/BallPlacementReceive.h"

namespace rtt::ai {

BallPlacementReceive::BallPlacementReceive(std::string name, bt::Blackboard::Ptr blackboard) : Receive(std::move(name), std::move(blackboard)) {}

bt::Node::Status BallPlacementReceive::onUpdate() {
    if (coach::g_pass.getRobotBeingPassedTo() != robot->get()->getId()) {
        return Status::Failure;
    }

    if (coach::g_pass.isPassed() && ball->get()->getVelocity().length() <= Constants::BALL_STILL_VEL()) {
        return Status::Success;
    }

    if ((ball->get()->getPos() - coach::g_ballPlacement.getBallPlacementPos()).length() < 0.25) {
        return Status::Success;
    }

    if (coach::g_pass.isPassed()) {
        command.set_dribbler(31);
        intercept();
    } else {
        Vector2 ballPlacementTarget = coach::g_ballPlacement.getBallPlacementPos();
        auto behindTargetPos = control::PositionUtils::getPositionBehindPositionToPosition(Constants::ROBOT_RADIUS(), ballPlacementTarget, ball->get()->getPos());

        moveToCatchPosition(behindTargetPos);
        if (isInPosition(behindTargetPos)) {
            coach::g_pass.setReadyToReceivePass(true);
        }
    }
    publishRobotCommand();
    return Status::Running;
}

void BallPlacementReceive::moveToCatchPosition(const Vector2 &position) {
    auto robotCommand = robot->getControllers().getNumTreePosController()->getRobotCommand(world, field, *robot, position);
    command.mutable_vel()->set_x(robotCommand.vel.x);
    command.mutable_vel()->set_y(robotCommand.vel.y);
    if (position.dist(robot->get()->getPos()) < 0.6) {
        command.set_w((Vector2(ball->get()->getPos()) - robot->get()->getPos()).angle());
    } else {
        command.set_w((position - robot->get()->getPos()).angle());
    }
}

// check if the robot is in the desired position to catch the ball
bool BallPlacementReceive::isInPosition(const Vector2 &behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->get()->getId(), true, ball->get()->getPos(), 0.3 * M_PI);
    bool isBehindTargetPos = behindTargetPos.dist(robot->get()->getPos()) < 0.10;
    return isBehindTargetPos && isAimedAtBall;
}

}  // namespace rtt::ai