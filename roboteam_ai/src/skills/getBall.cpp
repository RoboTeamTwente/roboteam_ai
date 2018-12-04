//
// Created by rolf on 04/12/18.
//

#include "getBall.h"
namespace c=rtt::ai::constants;

namespace rtt {
namespace ai {

//TODO: do obstacle checking and return fail if there is an obstacle in the way.
//GetBall turns the robot to the ball and softly approaches with dribbler on in an attempt to get the ball.
GetBall::GetBall(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}
std::string GetBall::node_name() {
    return "GetBall";
}
void GetBall::initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("GetBall Initialize -> robot does not exist in world");
            return;
        }
    }
    else {
        ROS_ERROR("GetBall Initialize -> ROLE WAITING!!");
        return;
    }
}
GetBall::Status GetBall::update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    }
    else {
        ROS_ERROR("GetBall Update -> robot does not exist in world");
    }
    ball = World::getBall(); //TODO: sanity checking if ball is actually there
    deltaPos = Vector2(robot.pos.x,robot.pos.y) - Vector2(ball.pos.x, ball.pos.y);
    currentProgress=checkProgression();
    if (currentProgress==TURNING){
        sendTurnCommand();
    }
    else if (currentProgress== APPROACHING){
        sendApproachCommand();
    }
    else if (currentProgress== DRIBBLING){
        sendDribblingCommand();
    }
    switch (currentProgress){
    case TURNING: return status::Running;
    case APPROACHING: return status::Running;
    case DRIBBLING: return status::Running;
    case SUCCESS: return status::Success;
    case FAIL: return status::Failure;
    }

}
void GetBall::terminate(Status s) {

}
bool GetBall::robothasBall() {
    //The ball is in an area defined by a cone from the robot centre, or from a rectangle in front of the dribbler
    Vector2 RobotPos = Vector2(robot.pos.x, robot.pos.y);
    Vector2 BallPos = Vector2(ball.pos.x, ball.pos.y);
    Vector2 dribbleLeft = RobotPos + Vector2(c::ROBOT_RADIUS, 0).rotate(robot.angle - c::DRIBBLER_ANGLE_OFFSET);
    Vector2 dribbleRight = RobotPos + Vector2(c::ROBOT_RADIUS, 0).rotate(robot.angle + c::DRIBBLER_ANGLE_OFFSET);

    std::vector<Vector2> drawPos = {RobotPos, dribbleLeft, dribbleRight,
                                    dribbleLeft + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle),
                                    dribbleRight + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle)};
    if (control::ControlUtils::pointInTriangle(BallPos, RobotPos, dribbleLeft, dribbleRight)) {
        return true;
    }
        // else check the rectangle in front of the robot.
    else
        return control::ControlUtils::pointInRectangle(BallPos, dribbleLeft, dribbleRight,
                dribbleRight + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle),
                dribbleLeft + Vector2(c::MAX_BALL_RANGE, 0).rotate(robot.angle));
}
void GetBall::sendTurnCommand() { }
void GetBall::sendApproachCommand(){ }

}//rtt
}//ai