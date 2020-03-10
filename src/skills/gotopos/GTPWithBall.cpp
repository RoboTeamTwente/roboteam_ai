
//
// Created by thijs on 26-4-19.
//

#include <coach/BallplacementCoach.h>
#include <skills/gotopos/GTPWithBall.h>

namespace rtt::ai {

GTPWithBall::GTPWithBall(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void GTPWithBall::onInitialize() {
    targetType = stringToTargetType(properties->getString("type"));
    updateTarget();
}

Skill::Status GTPWithBall::onUpdate() {
    updateTarget();
    command = robot->getControllers().getBallHandlePosController()->getRobotCommand(robot->get()->getId(), targetPos, targetAngle).makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

void GTPWithBall::onTerminate(Skill::Status s) {
    command.set_dribbler(0);
    command.mutable_vel()->set_x(0);
    command.mutable_vel()->set_y(0);
    command.set_w(robot->get()->getAngle());
    publishRobotCommand();
}

GTPWithBall::TargetType GTPWithBall::stringToTargetType(const std::string &string) { return ballPlacement; }

void GTPWithBall::updateTarget() {
    switch (targetType) {
        default:
            return;
        case ballPlacement: {
            targetPos = coach::g_ballPlacement.getBallPlacementPos();
            Vector2 delta = (targetPos - ball->get()->getPos());
            if (fabs(robot->get()->getAngle() - delta.toAngle()) < M_PI_2) {
                targetAngle = delta;
            } else {
                targetAngle = delta + M_PI;
            }
            return;
        }
    }
}

}  // namespace rtt::ai