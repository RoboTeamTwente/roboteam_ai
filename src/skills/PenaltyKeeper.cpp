//
// Created by rolf on 23-4-19.
//

#include "skills/PenaltyKeeper.h"
#include <control/ControlUtils.h>
#include <interface/api/Output.h>
#include <world/Ball.h>
#include <world/FieldComputations.h>
#include <world/Robot.h>
#include <world/World.h>

namespace rtt::ai {
PenaltyKeeper::PenaltyKeeper(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {}

void PenaltyKeeper::onInitialize() {
    goalLine = getGoalLine();
    state = WAITING;
    firstBallPos = world->getBall()->getPos();
    preparation = properties->getBool("prepare");
    gtp.setAutoListenToInterface(false);
}

PenaltyKeeper::Status PenaltyKeeper::onUpdate() {
    state = updateState(state);
    if (preparation) {
        state = WAITING;
    }
    switch (state) {
        case WAITING: {
            sendWaitCommand();
            break;
        }
        case BALLSHOT: {
            sendInterceptCommand();
            break;
        }
    }
    return Status::Running;
}
PenaltyKeeper::PenaltyState PenaltyKeeper::updateState(PenaltyState currentState) {
    if (currentState == WAITING) {
        // ballShotTicks=0;
        if (isBallShot()) {
            /*
            initialPos=robot->pos;
            initialVel=robot->vel;
            */
            return BALLSHOT;
        }
        return WAITING;
    } else if (currentState == BALLSHOT) {
        // ballShotTicks++;
        // prints for testing: easy to measure delay/effectiveness of our strategy
        // std::cout<<"BallPtr speed: "<<world::world->getBall()->vel<<std::endl;
        // std::cout<<"Pos diff(m) :" << (robot->pos-initialPos).length() << " Vel diff : " << (robot->vel-initialVel).length() <<" tick: "<<ballShotTicks<<std::endl;
        if (isBallShot()) {
            ballNotShotTicks = 0;
        } else {
            ballNotShotTicks++;
        }
        if (ballNotShotTicks > 3) {
            return WAITING;
        }
        return BALLSHOT;
    }
    return WAITING;
}
Vector2 PenaltyKeeper::computeDefendPos() {
    auto attacker = world::world->getRobotClosestToBall(THEIR_ROBOTS);
    // we check the line defined by attacker's centre and the ball position
    Vector2 middle = (goalLine.start + goalLine.end) * 0.5;

    if (attacker) {
        Vector2 beginPos = attacker->pos;
        Vector2 endPos = attacker->pos + (world::world->getBall()->getPos() - attacker->pos).stretchToLength((*field).getFieldLength());

        // we estimate we can move the robot about 20 cm during the shot and the opponent cannot shoot perfectly within 5 cm.
        double maxMoveDist = ((*field).getGoalWidth() - Constants::ROBOT_RADIUS()) / 2 - 0.2;
        LineSegment shootLine(beginPos, endPos);
        Line goalKeepingLine(goalLine.start, goalLine.end);
        auto intersection = goalKeepingLine.intersects(shootLine);
        if (intersection) {
            if (intersection->y > maxMoveDist) {
                return Vector2(middle.x, 0.4 * maxMoveDist);
            } else if (intersection->y < -maxMoveDist) {
                return Vector2(middle.x, -0.4 * maxMoveDist);
            }
            return *intersection * 0.4 + middle * 0.6;
        }
    }
    return middle;
}

Vector2 PenaltyKeeper::interceptBallPos() {
    Vector2 startBall = world::world->getBall()->getPos();
    Vector2 endBall = world::world->getBall()->getPos() + world::world->getBall()->getVel().stretchToLength(100);
    Vector2 predictedShotLocation = control::ControlUtils::twoLineIntersection(startBall, endBall, goalLine.start, goalLine.end);
    double margin = 0.05;  // m next to the goal
    if (predictedShotLocation.y <= (*field).getGoalWidth() * 0.5 + margin && predictedShotLocation.y >= -(*field).getGoalWidth() * 0.5 - margin) {
        return predictedShotLocation;
    }
    return (goalLine.start + goalLine.end) * 0.5;
}

void PenaltyKeeper::sendWaitCommand() {
    gtp.setAutoListenToInterface(false);

    gtp.updatePid(interface::Output::getKeeperPid());

    Vector2 targetPos = computeDefendPos();

    Vector2 delta = gtp.getRobotCommand(world, field, robot, targetPos).vel;
    command.mutable_vel()->set_x(delta.x);
    command.mutable_vel()->set_y(delta.y);
    command.set_w(M_PI_2);
    publishRobotCommand();
}

void PenaltyKeeper::sendInterceptCommand() {
    gtp.setAutoListenToInterface(false);
    gtp.updatePid({5.2, 0.0, 0.2});

    Vector2 interceptPos = interceptBallPos();
    Vector2 delta = gtp.getRobotCommand(world, field, robot, interceptPos).vel;
    command.mutable_vel()->set_x(delta.x);
    command.mutable_vel()->set_y(delta.y);
    command.set_w(M_PI_2);
    publishRobotCommand();
}

Line PenaltyKeeper::getGoalLine() {
    Line originalLine = FieldComputations::getGoalSides(*field, true);
    double forwardX = originalLine.start.x + Constants::KEEPER_PENALTY_LINE_MARGIN();
    originalLine.start.x = forwardX;
    originalLine.end.x = forwardX;
    return originalLine;
}

bool PenaltyKeeper::isBallShot() { return world::world->getBall()->getVel().x < -0.2; }

void PenaltyKeeper::onTerminate(rtt::ai::Skill::Status s) {
    state = WAITING;
    ballNotShotTicks = 0;
    goalLine = getGoalLine();
}
}  // namespace rtt::ai
