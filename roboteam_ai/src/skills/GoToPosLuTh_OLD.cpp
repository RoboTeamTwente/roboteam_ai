//
// Created by thijs on 19-11-18.
//

#include "GoToPosLuTh_OLD.h"
#include <random>  //random numbers..

namespace rtt {
namespace ai {

/// GoToPosLuTh: obstacle avoidance following Lukas & Thijs principles
GoToPosLuTh_OLD::GoToPosLuTh_OLD(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Called when the Skill is Initialized
void GoToPosLuTh_OLD::onInitialize() {
    drawInterface = properties->getBool("drawInterface");
    goToBall = properties->getBool("goToBall");
    random = properties->getBool("random");

    if (properties->hasVector2("Position")) {
        targetPos = properties->getVector2("Position");
    }
    else {
        ROS_ERROR("GoToPosLuTh_OLD Initialize -> No good X or Y set in properties");
        currentProgress = Progression::FAIL;
    }
}

/// Called when the Skill is Updated
GoToPosLuTh_OLD::Status GoToPosLuTh_OLD::onUpdate() {
    displayData.clear();

    if (goToBall) {
        auto ball = World::getBall();
        targetPos = ball.pos;
    }
    else if (random) {
        const roboteam_msgs::GeometryFieldSize &field = Field::get_field();
        const double &length = field.field_length;
        const double &width = field.field_width;
        int randomX = std::rand();
        int randomY = std::rand();

        random = false;
        targetPos = {randomX*4.65661288e-10*length - length*0.5, randomY*4.65661288 - 10*width - width*0.5};
    }

    // See if the progress is a failure
    if (currentProgress == Progression::FAIL) {
        return Status::Failure;
    }


//  ____________________________________________________________________________________________________________________

    // Now check the progress we made
    currentProgress = checkProgression();
    // Send a move command
    sendMoveCommand();

    switch (currentProgress) {

        // Return the progression in terms of status
        case ON_THE_WAY:
            return Status::Running;
        case DONE:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }

    return Status::Failure;
}

/// Called when the Skill is Terminated
void GoToPosLuTh_OLD::onTerminate(Status s) {
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 0;
    command.w = 0;

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
}

bool GoToPosLuTh_OLD::checkTargetPos(Vector2 pos) {
    // TODO: actually check
    return true;
}

void GoToPosLuTh_OLD::sendMoveCommand() {

    if (! checkTargetPos(targetPos)) {
        ROS_ERROR("Target position is not correct GoToPosLuTh_OLD");
        return;
    }

    ros::Time begin = ros::Time::now();

    numRobot me;
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    calculateNumericDirection(me, command);

    ros::Time end = ros::Time::now();
    double timeTaken = (end - begin).toSec();
    std::cout << "calculation: " << timeTaken*1000 << " ms" << std::endl;

    displayData.insert(displayData.end(), me.posData.begin(), me.posData.end());
    publishRobotCommand(command);
}

GoToPosLuTh_OLD::Progression GoToPosLuTh_OLD::checkProgression() {

    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    Vector2 deltaPos = {dx, dy};

    double maxMargin = 0.3;                        // max offset or something.

    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

bool GoToPosLuTh_OLD::calculateNumericDirection(numRobot &me, roboteam_msgs::RobotCommand &command) {

    me.id = robot->id;
    me.pos = robot->pos;
    me.vel = robot->vel;
    me.targetPos = targetPos;
    me.angle = robot->angle;
    int startIndex = 0;
    if (me.vel.length() > 10.0) return false;

    if (! tracePath(me, startIndex, targetPos, false)) {
        float angularVel = 0;

#define GRSIM
#ifdef OLDGRSIM
        command.use_angle = 0;
        angularVel = (float) Control::calculateAngularVelocity(robot.angle, angle);
#endif
#ifdef GRSIM
        command.use_angle = 1;
        if (! me.posData.empty()) {
            angularVel = (float) (me.posData.back() - me.posData.front()).angle();
        }
#endif
        command.w = angularVel;

        me.pos = robot->pos;
        auto world = World::get_world();
        Vector2 closestBot = getClosestRobotPos(world, me);
        if (me.isCollision(closestBot)) {
            Vector2 delta = closestBot - me.pos;
            command.x_vel = static_cast<float>(- delta.normalize().x*me.maxVel);
            command.y_vel = static_cast<float>(- delta.normalize().y*me.maxVel);
        }
        else if (me.velData.size() > 10) {
            command.x_vel = static_cast<float>(me.velData[2].x);
            command.y_vel = static_cast<float>(me.velData[2].y);
        }
        command.x_vel *= - 1;
        command.y_vel *= - 1;
        return true;
    }
//  ____________________________________________________________________________________________________________________

    std::cout << "robot travel time : " << me.t <<
              std::endl;

    int maxDTimesX = (int) (round(0.1/me.dt));
    int dTimesX;
    if (me.posData.empty()) {
        return true;
    }
    else if (static_cast<int>(me.posData.size()) > maxDTimesX) {
        dTimesX = maxDTimesX;
    }
    else {
        dTimesX = static_cast<int>(me.posData.size() - 1);
    }
    auto absXVel = static_cast<float>(me.velData[dTimesX].x);
    auto absYVel = static_cast<float>(me.velData[dTimesX].y);

    command.x_vel = - absXVel;
    command.y_vel = - absYVel;
    return false;
}

bool GoToPosLuTh_OLD::tracePath(numRobot &me, int &startIndex, Vector2 target, bool semiPath) {
    auto world = World::get_world();
    while (! me.isCollision(target)) {

        me.t = me.posData.size()*me.dt;
        // get the target velocity towards the direction we want to go
        if (semiPath) {
            me.targetVel = me.getDirection(target)*me.maxVel;
        }
        else {
            me.targetVel = me.getDirection()*me.maxVel;
        }

        // change acceleration towards the target velocity
        me.acc = (me.targetVel - me.vel).normalize()*me.maxAcc;
        // if the current velocity is away (5>90 degrees) from the target
        auto dAngle = static_cast<float>(me.vel.angle() - me.getDirection().angle());
        if (std::abs(dAngle) > M_PI_2) {
            me.acc = (me.acc.normalize() - me.vel.normalize())*me.maxAcc;
        }

        // change my current velocity and position following a numeric euler model
        me.pos = me.pos + me.vel*me.dt;
        me.vel = me.vel + me.acc*me.dt;

        // save position and velocity-data
        me.velData.push_back(me.vel);
        me.posData.push_back(me.pos);

        Vector2 closestBot = getClosestRobotPos(world, me);
        if (me.isCollision(closestBot)) {
            // in calculating a part of the path we do not want to collide
            if (semiPath) {
                return false;
            }
                // if we are in the main loop try to find an alternative path
            else {
                if (! avoidObject(me, startIndex, true)) {
                    return false;
                }

            }
        }

        if (++ me.totalCalculations > 5000) {
            return false;
        }
    }

    // yay, we finished the path! - hopefully..
    return true;
}

bool GoToPosLuTh_OLD::avoidObject(numRobot &me, int &startIndex, bool firstTry) {
    std::vector<Vector2> oldPosData = me.posData;
    std::vector<Vector2> oldVelData = me.velData;
    if (me.posData.size() - startIndex > 2) {
        Vector2 collisionPoint = me.pos;
        Vector2 startPos = me.posData[startIndex];
        Vector2 startVel = me.velData[startIndex];

        std::vector<std::vector<Vector2>> allPosData;
        std::vector<std::vector<Vector2>> allVelData;
        int nTries = 10;
        for (int tt = 1 - nTries; tt < nTries; tt ++) {

            std::vector<Vector2> newPosData(me.posData.begin(), me.posData.begin() + startIndex);
            me.posData = newPosData;
            std::vector<Vector2> newVelData(me.velData.begin(), me.velData.begin() + startIndex);
            me.velData = newVelData;

            me.pos = startPos;
            me.vel = startVel;
            if (firstTry) {
                Vector2 delta = (collisionPoint - startPos);
                if (delta.length() < 2.0) {
                    Vector2 quarter = {0.25, 0.25};
                    delta = delta + delta.normalize()*quarter;
                }
                Vector2 sideLength = {tt*delta.y/nTries, - tt*delta.x/nTries};
                Vector2 target = startPos + delta + sideLength;

                displayData.push_back(target);

                if (tracePath(me, startIndex, target, true)) {
                    if (static_cast<int>(me.posData.size()) > startIndex + 2) {
                        allPosData.push_back(me.posData);
                        allVelData.push_back(me.velData);

                        displayData.insert(displayData.end(), me.posData.begin(), me.posData.end());
                    }
                }
            }
            else {
                Vector2 half = {0.5, 0.5};
                Vector2 delta = (collisionPoint - startPos);
                Vector2 middle = half*(collisionPoint + startPos);
                Vector2 target = middle + delta.rotate(tt*M_PI/(nTries*1.0));

                displayData.push_back(target);

                if (tracePath(me, startIndex, target, true)) {

                    if (static_cast<int>(me.posData.size()) > startIndex + 2) {
                        allPosData.push_back(me.posData);
                        allVelData.push_back(me.velData);
                    }

                }
                displayData.insert(displayData.end(), me.posData.begin(), me.posData.end());
            }
        }
        if (! allPosData.empty()) {
            double distance = 0;
            int bestIndex = 0;
            auto world = World::get_world();
            for (int i = 0; i < static_cast<int>(allPosData.size()) - 1; i ++) {
                auto &posData = allPosData[i];
                Vector2 closestBot = getClosestRobotPos(world, me);
                //double distToTarget = (me.targetPos - posData.back()).length();
                double distToRobot = (closestBot - posData.back()).length();
                if (distToRobot < distance) {
                    distance = distToRobot;
                    bestIndex = i;
                }
            }
            me.posData = allPosData[bestIndex];
            me.velData = allVelData[bestIndex];

            if (static_cast<int>(me.posData.size()) < startIndex + 2) {
                me.posData = oldPosData;
                me.velData = oldVelData;
                return false;
            }
            me.pos = me.posData.back();
            me.vel = me.velData.back();
            startIndex = static_cast<int>(me.posData.size());
            return true;
        }
        else {
            return avoidObject(me, startIndex, false);

        }
    }
    else {
        me.posData = oldPosData;
        me.velData = oldVelData;
        return false;
    }
}

Vector2 GoToPosLuTh_OLD::getClosestRobotPos(const roboteam_msgs::World &world, numRobot &me) {
    Vector2 closestPos;
    double distance = 99999999;
    for (auto &bot : world.us) {
        if (static_cast<int>(bot.id) != me.id) {
            Vector2 botPos = {bot.pos.x + bot.vel.x*me.t, bot.pos.y + bot.vel.y*me.t};
            double deltaPos = (me.pos - botPos).length();
            if (deltaPos < distance) {
                closestPos = bot.pos;
                distance = deltaPos;
            }
        }
    }
    for (auto &bot : world.them) {
        Vector2 botPos = {bot.pos.x + bot.vel.x*me.t, bot.pos.y + bot.vel.y*me.t};
        double deltaPos = (me.pos - botPos).length();
        if (deltaPos < distance) {
            closestPos = bot.pos;
            distance = deltaPos;
        }

    }
    return closestPos;
}

} // ai
} // rtt