//
// Created by robzelluf on 1/22/19.
//

#include <control/ballHandling/BallHandlePosControl.h>
#include <coach/PassCoach.h>
#include <coach/BallplacementCoach.h>
#include <interface/api/Input.h>
#include "skills/Receive.h"
#include "roboteam_utils/Polygon.h"
#include "roboteam_utils/Line.h"
#include <control/ControlUtils.h>
#include <world/WorldData.h>

namespace rtt {
namespace ai {

Receive::Receive(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Receive::onInitialize() {
    readyToPassSet = false;
    canMoveInDefenseArea=properties->getBool("canMoveInDefenseArea");
}

Receive::Status Receive::onUpdate() {
    if (world->robotHasBall(robot->id, true) || wasSuccessFull) {
        wasSuccessFull = true;
        return Status::Success;
    }

    if (coach::g_pass.isPassed()) {
        // Check if the ball was deflected
        if (passFailed()) {
            publishRobotCommand();
            return Status::Failure;
        }

        intercept();
        if ((ball->getPos() - robot->pos).length() < 1.0) {
            command.set_dribbler(31);
        }
    } else {
        command.set_w((ball->getPos() - robot->pos).toAngle().getAngle());
    }

    // Check if robot is in position, otherwise turn towards ball
    if (isInPosition()) {
        if (!readyToPassSet) {
            readyToPassSet = true;
            coach::g_pass.setReadyToReceivePass(true);
        }
    }

    publishRobotCommand();
    return Status::Running;

}

void Receive::onTerminate(Status s) {
//    readyToPassSet = false;
//    currentProgress = POSITIONING;
//    if (passFailed() || coach::g_pass.getRobotBeingPassedTo() != robot->id) {
//        coach::g_pass.resetPass(robot->id);
//    }
}


// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 Receive::computeInterceptPoint(const Vector2& startBall, const Vector2& endBall) {
    double defenseAreaMargin = 0.3;
    double outOfFieldMargin = -Constants::ROBOT_RADIUS();
    return control::ControlUtils::getInterceptPointOnLegalPosition(
            robot->pos, {startBall, endBall}, false, false, defenseAreaMargin, outOfFieldMargin);
}
// check if the robot is in the desired position to catch the ball
bool Receive::isInPosition(const Vector2& behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->id, true, ball->getPos(), 0.3*M_PI);
    return isAimedAtBall;
}

void Receive::intercept() {
    ball = world->getBall();
    double ballAngle = (ball->getPos() - robot->pos).toAngle().getAngle();

    ballStartPos = ball->getPos();
    ballStartVel = ball->getVel();
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_RECEIVE_TIME();
    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities;

    if ((interceptPoint - robot->pos).length() > 1.0) {
        velocities = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, interceptPoint).vel;
        if(control::ControlUtils::clearLine(robot->pos, interceptPoint, world->getWorld(), 1)) {
            velocities = velocities * 1.2;
        }
    } else {
        velocities = robot->getBasicPosControl()->getRobotCommand(world, field, robot, interceptPoint).vel;
    }
    command.mutable_vel()->set_x(static_cast<float>(velocities.x));
    command.mutable_vel()->set_y(static_cast<float>(velocities.y));
    command.set_w(ball->getVel().stretchToLength(-1).toAngle());

    interface::Input::drawData(interface::Visual::INTERCEPT, {ballStartPos, ballEndPos}, Qt::darkCyan, robot->id, interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::INTERCEPT, {interceptPoint}, Qt::cyan, robot->id, interface::Drawing::DOTS, 5, 5);
}

bool Receive::passFailed() {
    return (ball->getVel().length() < 0.3);
}


bool Receive::ballDeflected() {
    Angle robotToBallAngle = (robot->pos - ball->getPos()).toAngle();
    Angle ballVelocityAngle = (ball->getVel()).toAngle();

    return abs(robotToBallAngle - ballVelocityAngle) > BALL_DEFLECTION_ANGLE;

}

} // ai
} // rtt
