#include <utility>

//
// Created by thijs on 12-12-18.
//

#include "ControlGoToPosLuTh.h"

namespace control {

ControlGoToPosLuTh::Command ControlGoToPosLuTh::goToPos(RobotPtr robot, Vector2 &target) {
    targetPos = target;
    Command command;


//    if (! checkTargetPos(targetPos)) {
//        ROS_ERROR("Target position is not correct GoToPosLuTh");
//        return ;
//    }

    //ros::Time begin = ros::Time::now();
    bool recalculate;
    command.id = robot->id;
    if (! me.posData.empty()) {
        auto robotPos = static_cast<Vector2>(robot->pos);
        recalculate = false;
        int currentIndex = 0;
        double distance = 999999;
        for (int i = 0; i < static_cast<int>(me.posData.size()); i ++) {
            me.pos = me.posData[i];
            me.t = me.posData.size()*me.dt;
            Vector2 closestBot = ControlUtils::getClosestRobot(me.pos, me.id, true, me.t);
            if (me.isCollision(closestBot)) {
                recalculate = true;
                break;
            }
            if (me.isCollision(robotPos, distance)) {
                currentIndex = i;
                distance = (robotPos - me.pos).length();
            }
        }
        if (distance < 0.2) {
            robotIndex = currentIndex;
        }
        else {
            recalculate = true;
        }

    }
    else {
        recalculate = true;
    }
    //recalculate = true; // TODO thijs crap, pls fix
    if (recalculate) {
        startTime = ros::Time::now();
        me.clear();
        bool nicePath = calculateNumericDirection(robot, me, command);
        robotQueue = {};

        //ros::Time end = ros::Time::now();
        //double timeTaken = (end - begin).toSec();
        //std::cout << "calculation: " << timeTaken*1000 << " ms" << std::endl;

        // display
        {
            std::vector<std::pair<rtt::Vector2, QColor>> displayColorData = {{{}, {}}};

            for (auto &displayAll : displayData) {
                displayColorData.emplace_back(displayAll, Qt::green);
            }
            for (auto &displayMe : me.posData) {
                displayColorData.emplace_back(displayMe, Qt::red);
            }
            displayData = {};
            drawCross(targetPos);
            for (auto displayTarget : displayData) {
                displayColorData.emplace_back(displayTarget, Qt::blue);
            }
            rtt::ai::interface::Drawer::setGoToPosLuThPoints(robot->id, displayColorData);
        }

        // send command
        if (nicePath) {

            auto toStep = static_cast<int>(round(0.5/me.dt));

            if (me.velData.size() > toStep) {
                command.use_angle = 1;
                command.x_vel = static_cast<float>((me.velData[toStep].normalize()*me.maxVel).x);
                command.y_vel = static_cast<float>((me.velData[toStep].normalize()*me.maxVel).y);
                command.w = static_cast<float>(me.velData[toStep].angle());
            }

//            command.use_angle = 0;
//            command.w = static_cast<float>(control::ControlUtils::calculateAngularVelocity(robot->angle, 0));
        }
        else {
            command.use_angle = 0;

            Vector2 dir = (targetPos - robot->pos).normalize();
            command.x_vel = static_cast<float>(dir.x*2.0f);
            command.y_vel = static_cast<float>(dir.y*2.0f);
            command.w = static_cast<float>(control::ControlUtils::calculateAngularVelocity(robot->angle, 0));
        }

    }
    else {

        double time = (ros::Time::now() - startTime).toSec();
        auto toStep = static_cast<int>(round(time/me.dt));
        int minStep = 10;
        //not PID
//        if (me.velData.size() > toStep && me.velData.size() > minStep) {
//            command.use_angle = 1;
//            command.x_vel = static_cast<float>((me.velData[toStep].normalize()*me.maxVel).x);
//            command.y_vel = static_cast<float>((me.velData[toStep].normalize()*me.maxVel).y);
//            command.w = static_cast<float>(me.velData[toStep].angle());
//        }

        //PID
        if (me.posData.size() < minStep) {
            me.clear();
            command.x_vel = 0;
            command.y_vel = 0;
            command.w = 0;
        }
        else {
            auto size = static_cast<int>(me.posData.size() - 1);
            while (size < toStep --);

            PID pid;
            if (! pidInit) {
                pidInit = true;
                pid.initialize(1.0/rtt::ai::constants::tickRate);
                pid.setParams(3.0, 0.1, 0.1, 0.0, 0.0, 0.0);
            }
            Vector2 pidPos = me.posData[toStep];
            Vector2 vel = pid.posControl(robot->pos, pidPos, robot->vel);
            command.x_vel = static_cast<float>(vel.x);
            command.y_vel = static_cast<float>(vel.y);
            command.w = static_cast<float>(me.posData[toStep].angle());

            command.use_angle = 1;
        }
    }
    return command;

}

bool ControlGoToPosLuTh::calculateNumericDirection(RobotPtr robot, NumRobot &me,
        roboteam_msgs::RobotCommand &command) {

    me.id = robot->id;
    me.pos = robot->pos;
    me.vel = robot->vel;
    me.targetPos = targetPos;
    me.finalTargetPos = targetPos;
    me.posData.push_back(me.pos);
    me.velData.push_back(me.vel);
    if (me.vel.length() > 10.0) return false;
    me.t = me.posData.size()*me.dt;

    Vector2 closestBot = ControlUtils::getClosestRobot(me.pos, me.id, true, me.t);
    if (me.isCollision(closestBot, 0.4f)) return false;

    bool noCollision = tracePath(me, targetPos);
    if (! noCollision) return false;

    return true;
}

bool ControlGoToPosLuTh::tracePath(NumRobot &numRobot, Vector2 target) {
    ros::Time begin = ros::Time::now();

    NumRobotPtr numRobotPtr = std::make_shared<NumRobot>(numRobot);
    robotQueue.push(numRobotPtr);
    while (! robotQueue.empty()) {
        ros::Time now = ros::Time::now();

        if ((now - begin).toSec()*1000 > 3) { // time > 3ms
            break;
        }

        NumRobotPtr me = robotQueue.top();
        if (me->isCollision(target)) {
            numRobot.posData = me->posData;
            numRobot.velData = me->velData;
            return true;
        }
        else if (me->isCollision(me->targetPos)) {
            me->startIndex = me->posData.size();
            me->targetPos = target;
        }

        if (calculateNextPoint(me)) robotQueue.push(me);
        else {      //Collision!! calculate new points

            auto minPoints = static_cast<int>(ceil(1.0f/(me->dt)));
            auto newDataPoints = static_cast<int>(me->posData.size() - me->startIndex);

            if (me->posData.empty() || me->velData.empty()) {
                // no data.. something went wrong somewhere. just continue as if nothing happend. :)
                continue;
            }
            else if (newDataPoints > minPoints) {
                // set the starting point 1.0f seconds (or closer) before the collision
                me->startIndex = me->posData.size() - minPoints;
            }
            Vector2 collisionPos = me->pos;
            Vector2 startPos = me->posData[me->startIndex];
            std::vector<Vector2> newTargets = me->getNewTargets(collisionPos, startPos);

            for (auto newTarget : newTargets) {
                drawCross(newTarget);
                NumRobotPtr newNumRobotPtr = me->getNewNumRobot(me, newTarget);
                robotQueue.push(newNumRobotPtr);
            }

        }

        robotQueue.pop();
    }
    return false;
}

bool ControlGoToPosLuTh::calculateNextPoint(NumRobotPtr me) {

    me->t = me->posData.size()*me->dt;
    Vector2 closestBot = ControlUtils::getClosestRobot(me->pos, me->id, true, me->t);
    if (me->isCollision(closestBot)) {
        return false;
    }

    // get the target velocity towards the direction we want to go
    me->targetVel = me->getDirection(me->targetPos)*me->maxVel;

    // change acceleration towards the target velocity
    me->acc = (me->targetVel - me->vel).normalize()*me->maxAcc;
    // if the current velocity is away (>90 degrees) from the target
    auto dAngle = static_cast<float>(me->vel.angle() - me->getDirection().angle());
    if (std::abs(dAngle) > M_PI_2) {
        me->acc = (me->acc.normalize() - me->vel.normalize())*me->maxAcc;
    }

    // change my current velocity and position using a numeric euler model
    me->vel = me->vel + me->acc*me->dt;
    me->pos = me->pos + me->vel*me->dt;

    // save position and velocity-data
    me->velData.push_back(me->vel);
    me->posData.push_back(me->pos);
    displayData.push_back(me->pos);
    return true;
}

void ControlGoToPosLuTh::drawCross(Vector2 &pos) {
    float dist = 0.0075f;
    for (int i = - 7; i < 8; i ++) {
        for (int j = - 1; j < 2; j += 2) {
            Vector2 data = pos + (Vector2) {dist*i, dist*j*i};
            displayData.push_back(data);
        }
    }
}

} // control
