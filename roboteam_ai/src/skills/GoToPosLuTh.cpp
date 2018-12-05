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
        ROS_ERROR("GoToPosLuTh Initialize -> ROLE WAITING!!");
        return;
    }
//  ____________________________________________________________________________________________________________________

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
    command.id = robot.id;
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
    command.id = robot.id;
    calculateNumericDirection(me, command);

    ros::Time end = ros::Time::now();
    double timeTaken = (end - begin).toSec();

    interface::Drawer::setGoToPosLuThPoints(robot.id, displayData);
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

bool GoToPosLuTh::calculateNumericDirection(NumRobot &me, roboteam_msgs::RobotCommand &command) {

    me.id = robot.id;
    me.pos = robot.pos;
    me.vel = robot.vel;
    me.targetPos = targetPos;
    me.angle = robot.angle;
    if (me.vel.length() > 10.0) return false;
    tracePath(me, targetPos);

    return true;
}

bool GoToPosLuTh::tracePath(NumRobot &numRobot, Vector2 &target) {

    std::queue<NumRobotPtr> robotQueue;
    NumRobotPtr numRobotPtr = std::make_shared<NumRobot>(numRobot);
    robotQueue.push(numRobotPtr);
    while (! robotQueue.empty()) {

        NumRobotPtr me = robotQueue.front();
        bool noCollision = calculateNextPoint(me);
        if (noCollision) robotQueue.push(me);
        else {
            return false;
        }
        if (me->isCollision(target)) return true;

        robotQueue.pop();
    }
}

bool GoToPosLuTh::calculateNextPoint(GoToPosLuTh::NumRobotPtr me) {
    me->t = me->posData.size()*me->dt;
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

    Vector2 closestBot = Control::getClosestRobot(me->pos, me->id, true, me->t);
    if (me->isCollision(closestBot)) {
        return false;
    }

    if (++ me->totalCalculations > 5000) {
        return false;
    }
    return true;
}

} // ai
} // rtt