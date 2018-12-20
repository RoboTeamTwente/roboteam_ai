//
// Created by rolf on 04/12/18.
//

#include "GetBall.h"

namespace rtt {
namespace ai {

//TODO: do obstacle checking and return fail if there is an obstacle in the way.
//GetBall turns the robot to the ball and softly approaches with dribbler on in an attempt to get the ball.
GetBall::GetBall(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

// Essentially a state transition diagram.
void GetBall::checkProgression() {
    if (deltaPos.length() > c::MAX_GETBALL_RANGE) {
        currentProgress = FAIL;
        return;
    }
    double angleDif = Control::angleDifference(robot->angle, deltaPos.angle());
    if (currentProgress == TURNING) {
        if (angleDif < c::ANGLE_SENS) {
            currentProgress = APPROACHING;
            return;
        }
    }
    else if (currentProgress == APPROACHING) {
        if (angleDif >= c::ANGLE_SENS) {
            currentProgress = TURNING;
            return;
        }
        if (! robotHasBall()) {
            return;
        }
        else {
            currentProgress = DRIBBLING;
            return;
        }
    }
    else if (currentProgress == DRIBBLING) {
        if (! robotHasBall()) {
            currentProgress = APPROACHING;
            count = 0;
            return;
        }
        count ++;
        if (count > c::POSSES_BALL_CYCLES) {
            currentProgress = SUCCESS;
            return;
        }
    }
    else if (currentProgress == FAIL || currentProgress == SUCCESS) {
        return;
    }
}
void GetBall::onInitialize() {
    currentProgress = TURNING;
    count = 0;
}
GetBall::Status GetBall::onUpdate() {
    ball = World::getBall(); //TODO: sanity checking if ball is actually there
    deltaPos = Vector2(ball.pos.x, ball.pos.y) - Vector2(robot->pos.x, robot->pos.y);
    checkProgression();

    if (currentProgress == TURNING) {
        sendTurnCommand();
    }
    else if (currentProgress == APPROACHING) {
        sendApproachCommand();
    }
    else if (currentProgress == DRIBBLING) {
        sendDribblingCommand();
    }
    switch (currentProgress) {
        case TURNING:
            return Status::Running;
        case APPROACHING:
            return Status::Running;
        case DRIBBLING:
            return Status::Running;
        case SUCCESS:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }

    return Status::Failure;
}

void GetBall::onTerminate(Status s) {
    sendDribblingCommand();
}
bool GetBall::robotHasBall() {
    //The ball is in an area defined by a cone from the robot centre, or from a rectangle in front of the dribbler
    Vector2 RobotPos = Vector2(robot->pos.x, robot->pos.y);
    Vector2 BallPos = Vector2(ball.pos.x, ball.pos.y);
    Vector2 dribbleLeft = RobotPos + Vector2(c::ROBOT_RADIUS, 0).rotate(robot->angle - c::DRIBBLER_ANGLE_OFFSET);
    Vector2 dribbleRight = RobotPos + Vector2(c::ROBOT_RADIUS, 0).rotate(robot->angle + c::DRIBBLER_ANGLE_OFFSET);

    std::vector<Vector2> drawPos = {RobotPos, dribbleLeft, dribbleRight,
                                    dribbleLeft + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot->angle),
                                    dribbleRight + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot->angle)};
    if (control::ControlUtils::pointInTriangle(BallPos, RobotPos, dribbleLeft, dribbleRight)) {
        return true;
    }
        // else check the rectangle in front of the robot.
    else
        return control::ControlUtils::pointInRectangle(BallPos, dribbleLeft, dribbleRight,
                dribbleRight + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot->angle),
                dribbleLeft + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot->angle));
}
void GetBall::sendTurnCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = (float) deltaPos.angle();
    publishRobotCommand(command);

}
void GetBall::sendApproachCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.x_vel = (float) deltaPos.normalize().x*c::GETBALL_SPEED;
    command.y_vel = (float) deltaPos.normalize().y*c::GETBALL_SPEED;
    command.w = (float) deltaPos.angle();
    publishRobotCommand(command);

}
void GetBall::sendDribblingCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 1;
    command.dribbler = 1;
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = (float) deltaPos.angle();
    publishRobotCommand(command);
}

}//rtt
}//ai