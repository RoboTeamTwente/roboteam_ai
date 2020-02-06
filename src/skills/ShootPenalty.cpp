//
// Created by baris on 11-3-19.
//

#include "skills/ShootPenalty.h"
#include "world/Ball.h"
#include "world/Robot.h"
#include "world/World.h"

namespace rtt::ai {

ShootPenalty::ShootPenalty(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void ShootPenalty::onInitialize() {
    tick = 0;

    genevaSet = false;
    genevaState = 5;
    gtp.setCanMoveInDefenseArea(true);
    gtp.setAutoListenToInterface(false);  // HACK HACK
    gtp.setCanMoveOutOfField(false);
    gtp.updatePid({1.0, 0.0, 0.2});
    lineP = 8.0;
    additionalBallDist = Vector2(0.05, 0.0);
    forcedKickOn = true;
    forcedKickRange = Constants::MAX_KICK_RANGE() - 0.01;
}
bt::Node::Status ShootPenalty::onUpdate() {
    if (!robot) return Status::Running;
    if (!genevaSet) {
        genevaState = determineGenevaState();
        genevaSet = true;
    }
    double ydiff = ball->getPos().y - robot->pos.y;
    double gain = ydiff * lineP;
    if (tick < genevaChangeTicks && ydiff < 0.03) {
        tick++;
        command.mutable_vel()->set_x(0);
        command.mutable_vel()->set_y(gain);
        command.set_w(0);
        command.set_geneva_state(genevaState);
        auto ball = world::world->getBall();
        if (ball) {
            ballPos = ball->getPos();
        }
    } else {
        if (ball && !FieldComputations::pointIsInDefenceArea(*field, ballPos, false, -0.1)) {
            Vector2 targetPos = world::world->getBall()->getPos() + additionalBallDist;
            if (FieldComputations::pointIsInDefenceArea(*field, ballPos, false, 0.2)) {
                auto cmd = gtp.getRobotCommand(world, field, robot, targetPos);
                command.mutable_vel()->set_x(cmd.vel.x);
                command.mutable_vel()->set_y(cmd.vel.y + gain);
            } else {
                auto cmd = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, targetPos);
                command.mutable_vel()->set_x(cmd.vel.x);
                command.mutable_vel()->set_y(cmd.vel.y);
            }
            command.set_w(0);
            command.set_kicker(true);
            command.set_chip_kick_vel(Constants::MAX_KICK_POWER());
            std::cout << robot->calculateDistanceToBall(ballPos) << std::endl;
            if (forcedKickOn || !robot->hasWorkingBallSensor()) {
                double dist = robot->calculateDistanceToBall(ballPos);
                if (dist != -1.0) {
                    command.set_chip_kick_forced(dist < forcedKickRange);
                } else {
                    std::cout << "HELP!" << std::endl;
                }
            }
            command.set_geneva_state(genevaState);
        }
    }
    command.set_geneva_state(genevaState);
    std::cout << "robotID " << robot->id << " " << genevaState << std::endl;
    publishRobotCommand();
    return Status::Running;
}

int ShootPenalty::determineGenevaState() {
    // determine the shortest position from where to kick the ball
    if (robot->pos.y < ball->getPos().y) {
        return genevaState = 1;
    }
    return genevaState = 5;
}

}  // namespace rtt::ai