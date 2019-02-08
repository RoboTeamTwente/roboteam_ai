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
    otherRobotID = Coach::pickOffensivePassTarget(robot->id, roleName);
    otherRobot = World::getRobotForId(static_cast<unsigned int>(otherRobotID), true);
    std::cout << "my ID: " << robot->id << "\nother ID: " << otherRobot->id << std::endl;
}

/// Called when the Skill is Updated
Pass::Status Pass::onUpdate() {
    if (! robot)
        return Status::Running;
    if (! World::getRobotForId(static_cast<unsigned int>(otherRobotID), true))
        return Status::Running;
    otherRobot = World::getRobotForId(static_cast<unsigned int>(otherRobotID), true);

    std::cout << fsm << std::endl;
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 0;
    goToType = GoToType::basic;
    switch (fsm) {
    case S1: {
        goToType = GoToType::luTh;
        if (IamNumber1)
            getBall();
        else
            targetPos = P2;

        if (IamNumber1 ? (
                Coach::isRobotBehindBallToPosition(0.5, otherRobot->pos, robot->pos) &&
                        (P2 - otherRobot->pos).length() < errorMargin) : (
                    Coach::isRobotBehindBallToPosition(0.5, robot->pos, otherRobot->pos) &&
                            (P2 - robot->pos).length() < errorMargin))
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S2: {
        if (IamNumber1)
            shootBall(command);
        else
            receiveBall(command, P2);
        if ((IamNumber1 ? (
                Coach::getRobotClosestToBall(true).get()->id == otherRobot->id) : (
                     Coach::getRobotClosestToBall(true).get()->id == robot->id)) &&
                ((Vector2) (ball->vel)).length() < 0.5f)

            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S3: {
        goToType = GoToType::luTh;
        if (! IamNumber1)
            getBall();
        else
            targetPos = P3;

        if (! IamNumber1 ? (
                Coach::isRobotBehindBallToPosition(0.5, otherRobot->pos, robot->pos) &&
                        (P3 - otherRobot->pos).length() < errorMargin) : (
                    Coach::isRobotBehindBallToPosition(0.5, robot->pos, otherRobot->pos) &&
                            (P3 - robot->pos).length() < errorMargin))
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S4: {
        if (! IamNumber1)
            shootBall(command);
        else
            receiveBall(command, P3);
        if ((! IamNumber1 ? (
                Coach::getRobotClosestToBall(true).get()->id == otherRobot->id) : (
                     Coach::getRobotClosestToBall(true).get()->id == robot->id)) &&
                ((Vector2) (ball->vel)).length() < 0.5f)

            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S5: {
        return Status::Success;
        goToType = GoToType::luTh;
        if (IamNumber1)
            getBall();
        else
            targetPos = P1;

        if (IamNumber1 ? (
                Coach::isRobotBehindBallToPosition(0.5, otherRobot->pos, robot->pos) &&
                        (P1 - otherRobot->pos).length() < errorMargin) : (
                    Coach::isRobotBehindBallToPosition(0.5, robot->pos, otherRobot->pos) &&
                            (P1 - robot->pos).length() < errorMargin))
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S6: {
        if (IamNumber1)
            shootBall(command);
        else
            receiveBall(command, P1);
        if ((IamNumber1 ? (
                Coach::getRobotClosestToBall(true).get()->id == otherRobot->id) : (
                     Coach::getRobotClosestToBall(true).get()->id == robot->id)) &&
                ((Vector2) (ball->vel)).length() < 0.5f)

            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S7: {
        goToType = GoToType::luTh;
        if (! IamNumber1)
            getBall();
        else
            targetPos = P2;

        if (! IamNumber1 ? (
                Coach::isRobotBehindBallToPosition(0.5, otherRobot->pos, robot->pos) &&
                        (P2 - otherRobot->pos).length() < errorMargin) : (
                    Coach::isRobotBehindBallToPosition(0.5, robot->pos, otherRobot->pos) &&
                            (P2 - robot->pos).length() < errorMargin))
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S8: {
        if (! IamNumber1)
            shootBall(command);
        else
            receiveBall(command, P2);
        if ((! IamNumber1 ? (
                Coach::getRobotClosestToBall(true).get()->id == otherRobot->id) : (
                     Coach::getRobotClosestToBall(true).get()->id == robot->id)) &&
                ((Vector2) (ball->vel)).length() < 0.5f)

            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S9: {
        goToType = GoToType::luTh;
        if (IamNumber1)
            getBall();
        else
            targetPos = P3;

        if (IamNumber1 ? (
                Coach::isRobotBehindBallToPosition(0.5, otherRobot->pos, robot->pos) &&
                        (P3 - otherRobot->pos).length() < errorMargin) : (
                    Coach::isRobotBehindBallToPosition(0.5, robot->pos, otherRobot->pos) &&
                            (P3 - robot->pos).length() < errorMargin))
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S10: {
        if (IamNumber1)
            shootBall(command);
        else
            receiveBall(command, P3);
        if ((IamNumber1 ? (
                Coach::getRobotClosestToBall(true).get()->id == otherRobot->id) : (
                     Coach::getRobotClosestToBall(true).get()->id == robot->id)) &&
                ((Vector2) (ball->vel)).length() < 0.5f)

            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S11: {
        goToType = GoToType::luTh;
        if (! IamNumber1)
            getBall();
        else
            targetPos = P1;

        if (! IamNumber1 ? (
                Coach::isRobotBehindBallToPosition(0.5, otherRobot->pos, robot->pos) &&
                        (P1 - otherRobot->pos).length() < errorMargin) : (
                    Coach::isRobotBehindBallToPosition(0.5, robot->pos, otherRobot->pos) &&
                            (P1 - robot->pos).length() < errorMargin))
            fsm = fsmPlusPlus(fsm);
        break;
    }
    case S12: {
        if (! IamNumber1)
            shootBall(command);
        else
            receiveBall(command, P1);
        if ((! IamNumber1 ? (
                Coach::getRobotClosestToBall(true).get()->id == otherRobot->id) : (
                     Coach::getRobotClosestToBall(true).get()->id == robot->id)) &&
                ((Vector2) (ball->vel)).length() < 0.5f)

            fsm = fsmPlusPlus(fsm);
        break;
    }
    }

    targetPos = control::ControlUtils::projectPositionToWithinField(targetPos);
    Vector2 velocity = goToPos.goToPos(robot, targetPos, goToType);
    command.x_vel = static_cast<float>(velocity.x);
    command.y_vel = static_cast<float>(velocity.y);
    if (command.use_angle == 0) {
        command.use_angle = 1;
        command.w = static_cast<float>((targetPos - robot->pos).angle());
    }
    publishRobotCommand(command);

    return Status::Running;

}

void Pass::receiveBall(roboteam_msgs::RobotCommand &command, const Vector2 &pos) {
    if (((Vector2) ball->vel).length() > 0.5f) {
        Vector2 a1 = ball->pos;
        Vector2 a2 = a1 + ball->vel;
        Vector2 b1 = robot->pos;
        Vector2 b2 = b1 + (Vector2) {- a2.y, a2.x};
        targetPos = control::ControlUtils::twoLineIntersection(a1, a2, b1, b2);
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) (ball->pos) - robot->pos).angle());
    }
    else {
        targetPos = pos;
    }
    goToType = GoToType::basic;

}

void Pass::shootBall(roboteam_msgs::RobotCommand &command) {
    Vector2 ballPos = ball->pos;
    Vector2 behindBall = Coach::getPositionBehindBallToGoal(0.5, true);
    Vector2 deltaBall = behindBall - ballPos;

    if (! Coach::isRobotBehindBallToGoal(0.5, true, robot->pos)) {
        targetPos = behindBall;
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) otherRobot->pos - ballPos).angle());
        goToType = GoToType::luTh;
    }

    if (((Vector2) ball->vel).length() < 0.5f) {
        targetPos = ball->pos;
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) otherRobot->pos - ballPos).angle());
    }
    else
        targetPos = robot->pos;
    if (Coach::doesRobotHaveBall(robot->id, true)) {
        command.use_angle = 1;
        command.w = static_cast<float>(((Vector2) otherRobot->pos - ballPos).angle());


        if (Coach::doesRobotHaveBall(robot->id, true, 0.15, 0.2)) {
            command.kicker = 1;
            command.kicker_vel = static_cast<float>(rtt::ai::Constants::MAX_KICK_POWER()*0.35f);
            command.kicker_forced = 1;
        }
    }
    goToType = GoToType::basic;
}

void Pass::getBall() {
    Vector2 ball = World::getBall()->pos;
    Vector2 behindBall = Coach::getPositionBehindBallToPosition(0.5, otherRobot->pos);
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
