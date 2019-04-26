//
// Created by robzelluf on 1/22/19.
//

#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>
#include "Receive.h"

namespace rtt {
namespace ai {

Receive::Receive(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Receive::onInitialize() {
    ballPlacement = properties->getBool("BallPlacement");
    isBallOnPassedSet = false;
    std::cerr << "Initialize pass for robot " << robot->id << std::endl;
}

Receive::Status Receive::onUpdate() {
    if (world::world->ourRobotHasBall(robot->id)) {
        return Status::Success;
    }

    if (coach::g_pass.getRobotBeingPassedTo() != robot->id) {
        return Status::Failure;
    }

    if (coach::g_pass.passTakesTooLong()) {
        std::cerr << "Pass takes too long" << std::endl;
        return Status::Failure;
    }

    if (ballPlacement) {
        Vector2 ballPlacementTarget = coach::g_ballPlacement.getBallPlacementPos();
        auto behindTargetPos = coach::g_generalPositionCoach.getPositionBehindPositionToPosition(
                Constants::ROBOT_RADIUS(),
                ballPlacementTarget,
                ball->pos);

        moveToCatchPosition(behindTargetPos);

        if (isInPosition(behindTargetPos)) {
            coach::g_pass.setReadyToReceivePass(true);
        }
    } else {
        // Check if robot is in position, otherwise turn towards ball
        if (isInPosition()) {
            coach::g_pass.setReadyToReceivePass(true);
        } else {
            command.w = static_cast<float>((Vector2(ball->pos) - robot->pos).angle());
            publishRobotCommand();
            return Status::Running;
        }
    }

    if (coach::g_pass.isPassed()) {
        // Remember the status of the ball at the moment of passing
        if(!isBallOnPassedSet) {
            ballOnPassed = ball;
            isBallOnPassedSet = true;
        }

        // Check if the ball was deflected
        if (isBallOnPassedSet && passFailed()) {
            std::cerr << "Robot " << robot->id << " failed to receive" << std::endl;
            command.w = -robot->angle;
            publishRobotCommand();
            return Status::Failure;
        }

        intercept();
        return Status::Running;
    }

    return Status::Running;
}

void Receive::onTerminate(Status s) {
    command.x_vel = 0;
    command.y_vel = 0;
    command.dribbler = 0;
    publishRobotCommand();

    //TODO: Remove temporary hack
    if (robot->id != -1) {
        coach::g_pass.resetPass();
    }
    std::cerr << "Terminate pass for robot " << robot->id << std::endl;
}


// Pick the closest point to the (predicted) line of the ball for any 'regular' interception
Vector2 Receive::computeInterceptPoint(const Vector2& startBall, const Vector2& endBall) {
    return Vector2(robot->pos).project(startBall, endBall);
}

// check if the robot is in the desired position to catch the ball
bool Receive::isInPosition(const Vector2& behindTargetPos) {
    bool isAimedAtBall = control::ControlUtils::robotIsAimedAtPoint(robot->id, true, ball->pos, 0.3*M_PI);

    if (ballPlacement) {
        bool isBehindTargetPos = behindTargetPos.dist(robot->pos) < 0.03;
        return isBehindTargetPos  && isAimedAtBall;
    }
    return isAimedAtBall;

}

void Receive::moveToCatchPosition(Vector2 position) {
    control::PosVelAngle pva = numTreeGtp.getPosVelAngle(robot, position);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, rtt::ai::Constants::MAX_VEL());
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);

    if (position.dist(robot->pos) < 0.6) {
        command.w = static_cast<float>((Vector2(ball->pos) - robot->pos).angle());
    } else {
        command.w = static_cast<float>((position - robot->pos).angle());

    }
    publishRobotCommand();
}

void Receive::intercept() {
    double ballAngle = ((Vector2) ball->pos - robot->pos).angle();

    ballStartPos = ball->pos;
    ballStartVel = ball->vel;
    ballEndPos = ballStartPos + ballStartVel * Constants::MAX_INTERCEPT_TIME();
    Vector2 interceptPoint = Receive::computeInterceptPoint(ballStartPos, ballEndPos);

    Vector2 velocities = basicGtp.getPosVelAngle(robot, interceptPoint).vel;

    velocities = control::ControlUtils::velocityLimiter(velocities);

    if (velocities.length() < 0.5) {
        velocities = velocities.stretchToLength(0.5);
    }

    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    command.w = ballAngle;

    publishRobotCommand();
}

bool Receive::passFailed() {
    //TODO: Remove print statements and make 1 big if statement
    if ((ball->vel.toAngle() - ballOnPassed->vel.toAngle()).getAngle() > BALL_DEFLECTION_ANGLE) {
        std::cerr << "Ball deflected" << std::endl;
        return true;
    }

    if (ball->vel.length() < 0.1) {
        std::cerr << "Ball going to slow" << std::endl;
        return true;
    }

//    if (receiverMissedBall()) {
//        std::cout << "Receiver missed ball!" << std::endl;
//        std::cout << ballOnPassed->pos << std::endl;
//        std::cout << ball->pos << std::endl;
//        std::cout << robot->pos << std::endl;
//
//        return true;
//    }
    return false;

}

bool Receive::receiverMissedBall() {
    return (ball->pos - ballOnPassed->pos).length() - (robot->pos - ballOnPassed->pos).length() >
           RECEIVER_MISSED_BALL_MARGIN;
}


} // ai
} // rtt
