//
// Created by rolf on 28/11/18.
//

#include "Dribble.h"
namespace rtt {
namespace ai {
Dribble::Dribble(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) { }

namespace c=rtt::ai::constants;
Dribble::Progression Dribble::checkProgression() {
    if (currentProgress == ON_THE_WAY) {
        if (! robotHasBall()) {
            return FAIL; }
        if (deltaPos.length() <= c::DRIBBLE_POSDIF) {
            if (forwardDirection)
            {
                stoppingAngle=(float) deltaPos.angle();
            }
            else{
                stoppingAngle=(float) deltaPos.rotate(M_PI).angle();
            }
            return STOPPED;
        }
        else { return ON_THE_WAY; }
    }
    else if (currentProgress == STOPPED) {
        count++;
        //ROS_WARN_STREAM("Stopped ticks #:" << count<<"/"<<maxTicks);
        if (! robotHasBall()) {
            return FAIL;
        }
        else if (count>=maxTicks) {
            return DONE;
        }
        else return STOPPED;
    }
    else if (currentProgress == DONE) {
        return DONE;
    }
    else if (currentProgress == FAIL) {
        return FAIL;
    }
    else if (currentProgress == WAITING) {
        return WAITING;
    }

}

bool Dribble::robotHasBall() {
    //The ball is in an area defined by a cone from the robot centre, or from a rectangle in front of the dribbler
    Vector2 RobotPos = Vector2(robot.pos.x, robot.pos.y);
    Vector2 BallPos = Vector2(ball.pos.x, ball.pos.y);
    Vector2 dribbleLeft = RobotPos + Vector2(c::ROBOT_RADIUS, 0).rotate(robot.angle - c::DRIBBLER_ANGLE_OFFSET);
    Vector2 dribbleRight = RobotPos + Vector2(c::ROBOT_RADIUS, 0).rotate(robot.angle + c::DRIBBLER_ANGLE_OFFSET);

    std::vector<Vector2> drawPos = {RobotPos, dribbleLeft, dribbleRight,
                                    dribbleLeft + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle),
                                    dribbleRight + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle)};
    interface.drawFrame(drawPos);
    if (control::ControlUtils::pointInTriangle(BallPos, RobotPos, dribbleLeft, dribbleRight)) {
        return true;
    }
        // else check the rectangle in front of the robot.
    else
        return control::ControlUtils::pointInRectangle(BallPos, dribbleLeft, dribbleRight,
                dribbleRight + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle),
                dribbleLeft + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle));
}
void Dribble::initialize() {
    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("Dribble Initialize -> robot does not exist in world");
            currentProgress = Progression::FAIL;
            return;
        }
    }
    else {
        ROS_ERROR("Dribble Initialize -> ROLE WAITING!!");
        currentProgress = Progression::FAIL;
        return;
    }
    //TODO: add failchecking if ball does not exist.

    ball = World::getBall();

    //if false, robot will dribble to the position backwards with the ball.
    forwardDirection = properties->getBool("forwardDirection");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else {
        ROS_ERROR("Dribble Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
        return;
    }
    if (properties->hasInt("maxTicks")) {
        maxTicks = properties->getInt("maxTicks");
    }
    else{
        ROS_ERROR("Dribble Initialize -> No maxTicks set!");
    }
    if (! Dribble::robotHasBall()) {
        ROS_ERROR("Dribble Initialize -> Robot does not have the ball!");
        currentProgress = Progression::FAIL;
        return;
    }
    currentProgress = Progression::ON_THE_WAY;
    count=0;

    stoppingAngle=robot.angle; // default to the current angle
}

Dribble::status Dribble::update() {
    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    }
    else {
        ROS_ERROR("Dribble Update -> robot does not exist in world");
    }
    ball = World::getBall(); //TODO: sanity checking if ball is actually there?

    if (currentProgress == Progression::FAIL) {
        return status::Failure;
    }
    else if (currentProgress == Progression::WAITING) {
        return status::Waiting;
    }

    deltaPos = targetPos - Vector2(ball.pos.x, ball.pos.y);
    currentProgress = checkProgression();

    if (currentProgress == STOPPED) {
        sendStopCommand();
    }
    else if (currentProgress == ON_THE_WAY) {
        sendMoveCommand();
    }

    switch (currentProgress) {
    case ON_THE_WAY: return status::Running;
    case STOPPED: return status::Running;
    case DONE: return status::Success;
    case FAIL: return status::Failure;
    default: return status::Waiting;
    }
}
void Dribble::terminate(Dribble::status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    if (forwardDirection) {
        command.w = (float) deltaPos.angle();
    }
    else {
        command.w = (float) deltaPos.rotate(M_PI).angle();
    }
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand(command);
}

void Dribble::sendMoveCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    if (forwardDirection) {
        command.w = (float) deltaPos.angle();
    }
    else {
        command.w = (float) deltaPos.rotate(M_PI).angle();
    }
    std::vector<Vector2> dposvec={deltaPos};
    interface.drawFrame(dposvec);
    command.dribbler = 1;
    command.x_vel = (float) deltaPos.normalize().x*c::DRIBBLE_SPEED;
    command.y_vel = (float) deltaPos.normalize().y*c::DRIBBLE_SPEED;
    publishRobotCommand(command);
}
void Dribble::sendStopCommand() {
    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w=stoppingAngle;
    command.dribbler = 0;
    command.x_vel = 0;
    command.y_vel = 0;
    publishRobotCommand(command);
}
std::string Dribble::node_name() { return "Dribble"; }

}
}
