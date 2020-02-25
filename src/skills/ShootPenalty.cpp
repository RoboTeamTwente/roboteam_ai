//
// Created by baris on 11-3-19.
//

#include <skills/ShootPenalty.h>
#include <world_new/FieldComputations.hpp>

namespace rtt::ai {

ShootPenalty::ShootPenalty(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void ShootPenalty::onInitialize() {
    tick = 0;

    genevaSet = false;
    genevaState = 5;
    robot->getControllers().getBasicPosController()->setCanMoveInDefenseArea(true);
    robot->getControllers().getBasicPosController()->setAutoListenToInterface(false);  // HACK HACK
    robot->getControllers().getBasicPosController()->setCanMoveOutOfField(false);
    robot->getControllers().getBasicPosController()->updatePid({1.0, 0.0, 0.2});
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
    double ydiff = ball->get()->getPos().y - robot->get()->getPos().y;
    double gain = ydiff * lineP;
    if (tick < genevaChangeTicks && ydiff < 0.03) {
        tick++;
        command.mutable_vel()->set_x(0);
        command.mutable_vel()->set_y(gain);
        command.set_w(0);
        command.set_geneva_state(genevaState);
        auto ball = world->getBall();
        if (ball) {
            ballPos = ball->get()->getPos();
        }
    } else {
        if (ball && !world_new::FieldComputations::pointIsInDefenceArea(*field, ballPos, false, -0.1)) {
            Vector2 targetPos = world->getBall()->get()->getPos() + additionalBallDist;
            if (world_new::FieldComputations::pointIsInDefenceArea(*field, ballPos, false, 0.2)) {
                auto cmd = robot->getControllers().getBasicPosController()->getRobotCommand(world, field, *robot, targetPos);
                command.mutable_vel()->set_x(cmd.vel.x);
                command.mutable_vel()->set_y(cmd.vel.y + gain);
            } else {
                auto cmd = robot->getControllers().getNumTreePosController()->getRobotCommand(world, field, *robot, targetPos);
                command.mutable_vel()->set_x(cmd.vel.x);
                command.mutable_vel()->set_y(cmd.vel.y);
            }
            command.set_w(0);
            command.set_kicker(true);
            command.set_chip_kick_vel(Constants::MAX_KICK_POWER());
            std::cout << robot->get()->getDistanceToBall() << std::endl;
            if (forcedKickOn || !robot->get()->isWorkingBallSensor()) {
                double dist = robot->get()->getDistanceToBall();
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
    std::cout << "robotID " << robot->get()->getId() << " " << genevaState << std::endl;
    publishRobotCommand();
    return Status::Running;
}

int ShootPenalty::determineGenevaState() {
    // determine the shortest position from where to kick the ball
    if (robot->get()->getPos().y < ball->get()->getPos().y) {
        return genevaState = 1;
    }
    return genevaState = 5;
}

}  // namespace rtt::ai