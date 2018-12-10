//
// Created by thijs on 04-12-18.
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
    robot = getRobotFromProperties(properties);

    drawInterface = properties->getBool("drawInterface");
    goToBall = properties->getBool("goToBall");
    random = properties->getBool("random");

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
    displayData.clear();

    updateRobot();
//  ____________________________________________________________________________________________________________________

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
        targetPos = {randomX*2.32830644e-10*length*2 - length*0.5, randomY*2.32830644e-10*width*2 - width*0.5};
        random = false;

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
    case ON_THE_WAY:return Status::Running;
    case DONE: return Status::Success;
    case FAIL: return Status::Failure;
    }

    return Status::Failure;
}

/// Called when the Skill is Terminated
void GoToPosLuTh::terminate(Status s) {

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 0;
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

    ros::Time begin = ros::Time::now();

    NumRobot me;
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    calculateNumericDirection(me, command);
    robotQueue = {};
    ros::Time end = ros::Time::now();
    double timeTaken = (end - begin).toSec();
    //std::cout << "calculation: " << timeTaken*1000 << " ms" << std::endl;

    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    for (auto displayAll : displayData) {
        displayColorData.emplace_back(displayAll, Qt::green);
    }
    for (auto displayMe : me.posData) {
        displayColorData.emplace_back(displayMe, Qt::red);
    }
    interface::Drawer::setGoToPosLuThPoints(robot->id, displayColorData);

    command.use_angle = 1;

//#define NOCOMMAND
#ifdef NOCOMMAND
        command.x_vel = 0.0f;
        command.y_vel = 0.0f;
        command.w = 0.0f;
#endif
    publishRobotCommand(command);
}

GoToPosLuTh::Progression GoToPosLuTh::checkProgression() {

    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    Vector2 deltaPos = {dx, dy};

    double maxMargin = 0.3;                        // max offset or something.

    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

bool GoToPosLuTh::calculateNumericDirection(NumRobot &me, roboteam_msgs::RobotCommand &command) {

    me.id = robot->id;
    me.pos = robot->pos;
    me.vel = robot->vel;
    me.targetPos = targetPos;
    me.finalTargetPos = targetPos;
    me.posData.push_back(me.pos);
    me.velData.push_back(me.vel);
    if (me.vel.length() > 10.0) return false;
    me.t = me.posData.size()*me.dt;

    Vector2 closestBot = Control::getClosestRobot(me.pos, me.id, true, me.t);
    if (me.isCollision(closestBot, 0.4f)) {
        return false;
    }
    tracePath(me, targetPos);

    if (me.velData.size() > 10) {
        if (abs(me.velData[2].angle() - me.velData[10].angle()) < 0.05f) {
            command.x_vel = static_cast<float>((me.velData[2].normalize()*me.maxVel).x);
            command.y_vel = static_cast<float>((me.velData[2].normalize()*me.maxVel).y);
            command.w = static_cast<float>(me.velData[2].angle());
            return true;
        } else {
            command.x_vel = static_cast<float>((me.velData[10].normalize()*me.maxVel).x);
            command.y_vel = static_cast<float>((me.velData[10].normalize()*me.maxVel).y);
            command.w = static_cast<float>(me.velData[10].angle());
        }
    }
    else {

        command.x_vel = 0.0f;
        command.y_vel = 0.0f;
        command.w = 0.0f;
        return false;
    }
}

bool GoToPosLuTh::tracePath(NumRobot &numRobot, Vector2 target) {

    NumRobotPtr numRobotPtr = std::make_shared<NumRobot>(numRobot);
    robotQueue.push(numRobotPtr);
    while (! robotQueue.empty()) {
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

bool GoToPosLuTh::calculateNextPoint(GoToPosLuTh::NumRobotPtr me) {

    me->t = me->posData.size()*me->dt;
    Vector2 closestBot = Control::getClosestRobot(me->pos, me->id, true, me->t);
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
    if (++ (me->totalCalculations) > 500) {
        std::cout << "careful: too many calculations!!: " << me->totalCalculations << std::endl;
    }

    return true;
}

void GoToPosLuTh::drawCross(Vector2 &pos) {
    float dist = 0.0075f;
    for (int i = - 7; i < 8; i ++) {
        for (int j = - 1; j < 2; j += 2) {
            Vector2 data = pos + (Vector2) {dist*i, dist*j*i};
            displayData.push_back(data);
        }
    }
}

} // ai
} // rtt