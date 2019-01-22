//
// Created by baris on 5-12-18.
//

#include "Pass.h"

namespace rtt {
namespace ai {

Pass::Pass(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Called when the Skill is Initialized
void Pass::onInitialize() {
    std::string roleName = properties->getString("ROLE");
    IamNumber1 = (roleName == "passOne");
    int otherRobotID = Coach::pickOffensivePassTarget(robot->id, roleName);
    otherRobot = World::getRobotForId(static_cast<unsigned int>(otherRobotID), true);
}

/// Called when the Skill is Updated
Pass::Status Pass::onUpdate() {
    if (! robot)
        return Status::Running;

    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    switch (fsm) {
    case S1: {
        if (IamNumber1)
            getBall();
        else
            targetPos = P2;

        if (Coach::isRobotBehindBallToRobot(0.3, true, robot->id, otherRobot->pos) &&
                (P2 - otherRobot->pos).length() < errorMargin)
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S2:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S3:{
        if (!IamNumber1)
            getBall();
        else
            targetPos = P3;

        if (Coach::isRobotBehindBallToRobot(0.3, true, otherRobot->id, robot->pos) &&
                (P3 - otherRobot->pos).length() < errorMargin)
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S4:{
        if (!IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S5:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S6:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S7:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S8:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S9:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S10:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S11:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    case S12:{
        if (IamNumber1) {
            shootBall(command);
        }
        else {
            receiveBall();
        }
        break;
    }
    }

    Vector2 velocity = goToPos.goToPos(robot, targetPos, goToType);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    command.w = static_cast<float>((targetPos-robot->pos).angle());
    publishRobotCommand(command);

    return Status::Running;

}

void Pass::receiveBall() {

}

void Pass::shootBall(roboteam_msgs::RobotCommand &command) {
    targetPos = ball->pos;
    if (Coach::doesRobotHaveBall(robot->id, true)) {
        command.kicker = 1;
        command.kicker_vel = static_cast<float>(rtt::ai::constants::MAX_KICK_POWER);
        command.kicker_forced = 1;
    }
    goToType = GoToType::basic;
}

void Pass::getBall() {
    Vector2 ball = World::getBall()->pos;
    Vector2 behindBall = Coach::getPositionBehindBallToGoal(0.5, true);
    Vector2 deltaBall = behindBall - ball;

    if (! Coach::isRobotBehindBallToPosition(0.5, otherRobot->pos, robot->pos)) {
        targetPos = behindBall;
        goToType = GoToType::luTh;
    }
    else {
        targetPos = ball;
        goToType = GoToType::basic;
    }
}

Pass::FSM012 Pass::fsmPlusPlus(FSM012 fsm) {
    std::cout << "current FSM: " << fsm << std::endl;
    switch (fsm) {
    case S1:return S2;
    case S2:return S3;
    case S3:return S4;
    case S4:return S5;
    case S5:return S6;
    case S6:return S7;
    case S7:return S8;
    case S8:return S9;
    case S9:return S10;
    case S10:return S11;
    case S11:return S12;
    case S12:return S1;
    }

    return S1;
}

} // ai
} // rtt
