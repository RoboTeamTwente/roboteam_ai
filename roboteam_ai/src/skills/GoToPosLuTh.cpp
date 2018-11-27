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
    }
    else {
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
    numRobot me;
    float xVel = 0, yVel = 0, angle = 0;
    bool collision = calculateNumericDirection(me, xVel, yVel, angle);

    interface.drawFrame(me.posData);

    roboteam_msgs::RobotCommand command;
    command.id = robot.id;
    command.use_angle = 1;

    auto angularVel = (float) Control::calculateAngularVelocity(robot.angle, angle);

    if (! collision) {

        command.x_vel = xVel;
        command.y_vel = yVel;
        command.w = angularVel;
    }
    else {

        command.x_vel = 0;
        command.y_vel = 0;
        command.w = angularVel;
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

bool GoToPosLuTh::calculateNumericDirection(numRobot &me, float &xVel, float &yVel, float &angle) {

    ros::Time begin = ros::Time::now();
    me.id = robot.id;
    me.pos = robot.pos;
    me.vel = robot.vel;
    me.targetPos = targetPos;
    me.angle = robot.angle;
    int startIndex = 0;
    if (me.vel.length() > 10.0) return false;

    tracePath(me, startIndex, me.targetPos, false);
//  ____________________________________________________________________________________________________________________

    std::cout << "robot travel time : " << me.t << std::endl;

    int maxDTimesX = (int) (round(0.1/me.dt));
    int dTimesX;
    if (me.posData.empty()) {
        xVel = 0;
        yVel = 0;
        angle = robot.angle;
        return false;
    }
    else if (me.posData.size() > maxDTimesX) {
        dTimesX = maxDTimesX;
    }
    else {
        dTimesX = static_cast<int>(me.posData.size() - 1);
    }
    auto absXVel = static_cast<float>(me.velData[dTimesX].x);
    auto absYVel = static_cast<float>(me.velData[dTimesX].y);

    xVel = absXVel;
    yVel = absYVel;
    angle = 0;

    ros::Time end = ros::Time::now();
    double timeTaken = (end - begin).toSec();
    std::cout << "calculation: " << timeTaken*1000 << " ms" << std::endl;

    return false;
}

bool GoToPosLuTh::tracePath(numRobot &me, int &startIndex, Vector2 &targetPos, bool semiPath) {
    auto world = World::get_world();
    auto us = world.us;
    auto them = world.them;
    while (! me.isCollision(targetPos)) {

        me.velData.push_back(me.vel);
        me.posData.push_back(me.pos);
        if (semiPath) {
            me.targetVel = me.getDirection(targetPos)*me.maxVel;
        }
        else {
            me.targetVel = me.getDirection()*me.maxVel;
        }
        me.acc = (me.targetVel - me.vel).normalize()*me.maxAcc;

        auto dAngle = static_cast<float>(me.vel.angle() - me.getDirection().angle());
        if (std::abs(dAngle) > M_PI_2) {
            me.acc = (me.acc.normalize() - me.vel.normalize())*me.maxAcc;
        }

        me.pos = me.pos + me.vel*me.dt;
        me.vel = me.vel + me.acc*me.dt;

        for (auto &ourBot : world.us) {
            if (ourBot.id != me.id) {
                ourBot.pos.x = ourBot.pos.x + ourBot.vel.x*me.dt;
                ourBot.pos.y = ourBot.pos.y + ourBot.vel.y*me.dt;

                if (me.isCollision(ourBot.pos)) {
                    if (semiPath) {
                        return false;
                    }
                    else {
                        avoidObject(me, startIndex);

                    }
                }
            }
        }
        for (auto &theirBot : world.them) {
            theirBot.pos.x = theirBot.pos.x + theirBot.vel.x*me.dt;
            theirBot.pos.y = theirBot.pos.y + theirBot.vel.y*me.dt;

            if (me.isCollision(theirBot.pos)) {
                if (semiPath) {
                    return false;
                }
                else {
                    avoidObject(me, startIndex);
                }

            }
        }

        me.t += me.dt;
    }
    return true;
}

void GoToPosLuTh::avoidObject(numRobot &me, int &startIndex) {

    auto nDataPoints = static_cast<int>(me.posData.size());
    if (nDataPoints > 2) {
        Vector2 collisionPoint = me.pos;
        Vector2 startPos = me.posData[startIndex];
        Vector2 startVel = me.velData[startIndex];

        std::vector<std::vector<Vector2>> allPosData;
        std::vector<std::vector<Vector2>> allVelData;

        for (int tt = - 5; tt < 6; tt ++) {

            std::vector<Vector2> newPosData(me.posData.begin(), me.posData.begin() + startIndex);
            me.posData = newPosData;
            std::vector<Vector2> newVelData(me.velData.begin(), me.velData.begin() + startIndex);
            me.velData = newVelData;
            me.pos = startPos;
            me.vel = startVel;

            Vector2 delta = (collisionPoint - startPos);
            Vector2 sideLength = {sin(tt*M_PI / 8)*delta.x, sin(tt*M_PI / 8)*delta.y};
            Vector2 target = collisionPoint + sideLength;
            if (tracePath(me, startIndex, target, true)) {
                allPosData.push_back(me.posData);
                allVelData.push_back(me.velData);
            }
        }
        unsigned long size = 1000000;
        int bestIndex = 0;
        for (int i = 0; i < allPosData.size()-1; i++) {
            auto &posData = allPosData[i];
            if (posData.size() < size) {
                size = posData.size();
                bestIndex = i;
            }
        }
        me.posData = allPosData[bestIndex];
        me.velData = allVelData[bestIndex];
        me.pos = me.posData.back();
        me.vel = me.velData.back();
        return;
    }
    else {
        return;
    }
}

} // ai
} // rtt