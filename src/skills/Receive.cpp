//
// Created by robzelluf on 1/22/19.
//

#include <coach/PassCoach.h>
#include <interface/api/Input.h>
#include <skills/Receive.h>

namespace rtt::ai {

Receive::Receive(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Receive::onInitialize() {
    readyToPassSet = false;
    canMoveInDefenseArea = properties->getBool("canMoveInDefenseArea");
}

Receive::Status Receive::onUpdate() {
    if (world.robotHasBall(robot->get()->getId(), true)) {
        return Status::Success;
    }

    if (coach::g_pass.getRobotBeingPassedTo() != robot->get()->getId()) {
        return Status::Failure;
    }

    if (coach::g_pass.isPassed()) {
        // Check if the ball was deflected
        if (passFailed()) {
            publishRobotCommand();
            return Status::Failure;
        }

        intercept();
        if ((ball->get()->getPos() - robot->get()->getPos()).length() < 1.0) {
            command.set_dribbler(31);
        }
    } else {
        command.set_w((ball->get()->getPos() - robot->get()->getPos()).toAngle().getAngle());
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
    readyToPassSet = false;
    if (passFailed() || coach::g_pass.getRobotBeingPassedTo() != robot->get()->getId()) {
        coach::g_pass.resetPass(robot->get()->getId());
    }
}

// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 Receive::computeInterceptPoint(const Vector2 &startBall, const Vector2 &endBall) {
    double defenseAreaMargin = 0.3;
    double outOfFieldMargin = -Constants::ROBOT_RADIUS();
    return control::ControlUtils::getInterceptPointOnLegalPosition(*field, robot->get()->getPos(), {startBall, endBall}, false, false, defenseAreaMargin, outOfFieldMargin);
}
// check if the robot is in the desired position to catch the ball
bool Receive::isInPosition(const Vector2 &behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->get()->getId(), true, ball->get()->getPos(), world, 0.3 * M_PI);
    return isAimedAtBall;
}

void Receive::intercept() {
    ball = world->getBall();

    ballStartPos = ball->get()->getPos();
    ballStartVel = ball->get()->getVelocity();
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_RECEIVE_TIME();
    Vector2 interceptPoint = computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities;

    if ((interceptPoint - robot->get()->getPos()).length() > 1.0) {
        velocities = robot->getControllers().getNumTreePosController()->getRobotCommand(robot->get()->getId(), interceptPoint).vel;
        if (control::ControlUtils::clearLine(robot->get()->getPos(), interceptPoint, world, 1)) {
            velocities = velocities * 1.2;
        }
    } else {
        velocities = robot->getControllers().getBasicPosController()->getRobotCommand(robot->get()->getId(), interceptPoint).vel;
    }
    command.mutable_vel()->set_x(static_cast<float>(velocities.x));
    command.mutable_vel()->set_y(static_cast<float>(velocities.y));
    command.set_w(ball->get()->getVelocity().stretchToLength(-1).toAngle());

    interface::Input::drawData(interface::Visual::INTERCEPT, {ballStartPos, ballEndPos}, Qt::darkCyan, robot->get()->getId(), interface::Drawing::LINES_CONNECTED);
    interface::Input::drawData(interface::Visual::INTERCEPT, {interceptPoint}, Qt::cyan, robot->get()->getId(), interface::Drawing::DOTS, 5, 5);
}

bool Receive::passFailed() { return (ball->get()->getVelocity().length() < 0.3); }

}  // namespace rtt::ai
