#include <utility>

//
// Created by thijs on 12-12-18.
//

#include "ControlGoToPosLuTh.h"

namespace rtt {
namespace ai {
namespace control {

void ControlGoToPosLuTh::clear() {
    me.clear();
}

Vector2 ControlGoToPosLuTh::goToPos(RobotPtr robot, Vector2 &target) {
    Vector2 velocityCommand;

    bool recalculate = false;
    double deltaTarget = (abs((target - targetPos).length()));
    double deltaPos = (abs((target - robot->pos).length()));

    if (deltaTarget > errorMargin && ! (deltaPos < errorMargin*4.0 && deltaTarget > errorMargin*2.0)) {
        recalculate = true;
    }
    else if (me.posData.size() > 4) {

        auto robotPos = static_cast<Vector2>(robot->pos);
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
        if (distance < 0.25) robotIndex = currentIndex;
        else recalculate = true;
    }
    else recalculate = true;

    // Calculate new path
    if (recalculate) {
        clear();
        startTime = ros::Time::now();

        targetPos = target;
        bool nicePath = calculateNumericDirection(robot, me);
        robotQueue = std::priority_queue<NumRobotPtr, std::vector<NumRobotPtr>, NumRobot::CustomCompare>();

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

        if (! nicePath) {
            std::cout << "No path found in gtpluth! " << std::endl;
            Vector2 dir = (targetPos - robot->pos).normalize();
            velocityCommand.x = static_cast<float>(dir.x*2.0f);
            velocityCommand.y = static_cast<float>(dir.y*2.0f);

        }

    }

    //PID
    double time = (ros::Time::now() - startTime).toSec();
    auto toStep = static_cast<int>(round(time/me.dt));
    int minStep = 10;
    if (toStep < minStep) toStep = minStep;

    if (me.posData.size() < minStep) {
        me.clear();

        Vector2 dir = (targetPos - robot->pos).normalize();
        velocityCommand.x = static_cast<float>(dir.x*2.0f);
        velocityCommand.y = static_cast<float>(dir.y*2.0f);
    }
    else {
        auto size = static_cast<int>(me.posData.size() - 1);
        while (size < toStep --);

        if (! pidInit) {
            pidInit = true;
            pid.setPID(3.0, 0, 0.5);
        }

        Vector2 pidPos = me.posData[toStep];
        Vector2 vel = pid.controlPIR(pidPos - robot->pos, robot->vel);
        if (vel.length() > 3.0)
            vel = vel.normalize()*3.0;
        velocityCommand.x = static_cast<float>(vel.x);
        velocityCommand.y = static_cast<float>(vel.y);

    }

    return velocityCommand;

}

bool ControlGoToPosLuTh::calculateNumericDirection(RobotPtr robot, NumRobot &me) {

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
    return noCollision;

}

bool ControlGoToPosLuTh::tracePath(NumRobot &numRobot, Vector2 target) {
    ros::Time begin = ros::Time::now();

    NumRobotPtr numRobotPtr = std::make_shared<NumRobot>(numRobot);
    robotQueue.push(numRobotPtr);
    while (! robotQueue.empty()) {
        ros::Time now = ros::Time::now();

        if ((now - begin).toSec()*1000 > constants::MAX_CALCULATION_TIME) {
            return false;
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
            (me->collisions) --;
            me->newDir = NumRobot::goMiddle;
        }

        if (calculateNextPoint(me)) robotQueue.push(me);
        else {      //Collision!! calculate new points

            auto minPoints = static_cast<int>(ceil(1.0f/(me->dt)));
            auto newDataPoints = static_cast<int>(me->posData.size() - 1 - me->startIndex);

            if (me->posData.empty() || me->velData.empty()) {
                // no data.. something went wrong somewhere. just continue as if nothing happend. :)
                continue;
            }
            else if (newDataPoints > minPoints) {
                // set the starting point 1.0f seconds (or closer) before the collision
                me->startIndex = me->posData.size() - minPoints;
            }
            if (me->startIndex == 0 && me->posData.size() > 1) me->startIndex = 1;
            Vector2 collisionPos = me->pos;
            Vector2 startPos = me->posData[me->startIndex];
            std::vector<std::pair<NumRobot::newDirections, Vector2>> newTargets = me->getNewTargets(collisionPos,
                    startPos);

            for (auto &newTarget : newTargets) {
                drawCross(newTarget.second);
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
} // ai
} // rtt

