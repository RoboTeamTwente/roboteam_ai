//
// Created by thijs on 19-11-18.
//

#include "GoToPosLuTh.h"

namespace rtt {
namespace ai {

/// GoToPosLuTh: obstacle avoidance following Lukas & Thijs principles
GoToPosLuTh::GoToPosLuTh(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Return name of GoToPosLuTh
std::string GoToPosLuTh::node_name() {
    return "GoToPosLuTh";
}

/// Called when the Skill is Initialized
void GoToPosLuTh::initialize() {

    if (properties->hasString("ROLE")) {
        std::string roleName = properties->getString("ROLE");
        robot.id = (unsigned int) dealer::findRobotForRole(roleName);
        if (World::getRobotForId(robot.id, true)) {
            robot = World::getRobotForId(robot.id, true).get();
        }
        else {
            ROS_ERROR("GoToPosLuTh Initialize -> robot does not exist in world");
            return;
        }
    }
    else {
        ROS_ERROR("GoToPosLuTh Initialize -> ROLE INVALID!!");
        return;
    }
//  ____________________________________________________________________________________________________________________

    goToBall = properties->getBool("goToBall");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else {
        ROS_ERROR("GoToPosLuTh Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
    }
}

/// Called when the Skill is Updated
GoToPosLuTh::Status GoToPosLuTh::update() {

    if (World::getRobotForId(robot.id, true)) {
        robot = World::getRobotForId(robot.id, true).get();
    } else {
        ROS_ERROR("GoToPosLuTh Update -> robot does not exist in world");
    }
//  ____________________________________________________________________________________________________________________

    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }
    else if (currentProgress == Progression::INVALID) {
        return Status::Invalid;
    }

//  ____________________________________________________________________________________________________________________

    // Now check the progress we made
    currentProgress = checkProgression();
    // Send a move command
    sendMoveCommand();

    switch (currentProgress) {

        // Return the progression in terms of status
    case ON_THE_WAY:return Status::Running;
    case DONE: return Status::Success;
    case FAIL: return Status::Failure;
    case INVALID: return Status::Invalid;
    }

    return Status::Failure;
}

/// Called when the Skill is Terminated
void GoToPosLuTh::terminate(Status s) {

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;
    command.w = 0;

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}



bool GoToPosLuTh::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

void GoToPosLuTh::sendMoveCommand() {

    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPosLuTh");
        return;
    }
    float xVel, yVel, angle;
    bool collision = calculateNumericDirection(xVel, yVel, angle);

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;

    if (!collision) {

        command.x_vel = 1.5;
        command.y_vel = 0;
        auto angularVel = (float)Control::calculateAngularVelocity(robot.angle, angle);
        command.w = angularVel;
    } else {

        command.w = 0.0f;
        command.x_vel = 0;
        command.y_vel = 0;
    }
    publishRobotCommand(command);
}

GoToPosLuTh::Progression GoToPosLuTh::checkProgression() {

    double dx = targetPos.x - robot.pos.x;
    double dy = targetPos.y - robot.pos.y;
    Vector2 deltaPos = {dx, dy};

    double maxMargin = 0.3;                        // max offset or something.

    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

bool GoToPosLuTh::calculateNumericDirection(float &x_vel, float &y_vel, float &angle) {

    ros::Time begin = ros::Time::now();


    struct numRobot {
      int id;                   //Robot id
      double angle;
      Vector2 pos;                  //Current x,y position in m
      Vector2 targetPos;            //Target position in m
      Vector2 vel;                  //Current x,y velocity in ms-1
      Vector2 targetVel;            //Target velocity in ms-1
      double maxVel = 2.0;          //Maximum velocity in ms-1
      Vector2 acc;                  //Current x,y acceleration in ms-2
      double maxAcc = 0.7;          //Maximum acceleration in ms-2
      std::vector<Vector2> posData; //Save the position data for visualization

      Vector2 getDirection() {
          return (targetPos-pos).normalize();
      }

      bool isCollision(Vector2 const &otherPos) {
          double minDistance = 0.2;
          return (std::abs((otherPos - pos).length()) < minDistance);
      }
    };
    numRobot me;
    me.id = robot.id;
    me.pos = robot.pos;
    me.vel = robot.vel;
    me.targetPos = targetPos;
    me.angle = robot.angle;

    double maxError = 0.1;
    const double dt = 0.005;
    double t = 0;

    auto world = World::get_world();
    auto us = world.us;
    auto them = world.them;

    while (std::abs((me.pos - me.targetPos).length()) > maxError) {

        t = t + dt;

        me.posData.push_back(me.pos);

        me.targetVel = me.getDirection()*me.maxVel;
        me.acc = (me.targetVel - me.vel).normalize() * me.maxAcc;

        me.pos = me.pos + me.vel*dt;
        me.vel = me.vel + me.acc*dt;

        for (auto &ourBot : world.us) {
            if (ourBot.id != me.id) {
                ourBot.pos.x = ourBot.pos.x + ourBot.vel.x*(float) dt;
                ourBot.pos.y = ourBot.pos.y + ourBot.vel.y*(float) dt;

                if (me.isCollision(ourBot.pos)) {
                    std::cerr << "me : " << me.id << " - potential collision with our robot with id : " << ourBot.id << std::endl;
                    return true;
                }
            }
        }
        for (auto &theirBot : world.them) {
            theirBot.pos.x = theirBot.pos.x + theirBot.vel.x*(float) dt;
            theirBot.pos.y = theirBot.pos.y + theirBot.vel.y*(float) dt;

            if (me.isCollision(theirBot.pos)) {
                std::cerr << "me : " << me.id << " - potential collision with their robot with id : " << theirBot.id << std::endl;
                return true;
            }
        }
    }
    std::cout << "robot travel time : " << t << std::endl;

    auto targetAngle = static_cast<float>((targetPos - robot.pos).angle());
    angle = targetAngle;

    ros::Time end = ros::Time::now();
    double timeTaken = (end-begin).toSec();
    std::cout << "calculation: " << timeTaken*1000 << " ms" << std::endl;
    for (auto &p : me.posData) {
        std::cout << "position: { " << p.x << ", " << p.y << " }" << std::endl;
    }

    return false;
}




} // ai
} // rtt