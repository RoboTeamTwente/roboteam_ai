//
// Created by baris on 8-4-19.
//

#include <skills/ActiveStop.h>

namespace rtt::ai {

int ActiveStop::attack = -1;

ActiveStop::ActiveStop(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) {}
void ActiveStop::onInitialize() {
    robot->getControllers().getNumTreePosController()->setAvoidBallDistance(0.8);

    if (attack == -1) {
        attack = robot->get()->getId();
        attacker = true;
    }
}

Skill::Status ActiveStop::onUpdate() {
    if (attacker) {
        targetPos = getOffensiveActivePoint(*field, *ball);
    } else {
        targetPos = getDefensiveActivePoint(*field, *ball);
    }

    if (robot->get()->getPos().dist(targetPos) > 0.3) {
        command.set_w(static_cast<float>((targetPos - robot->get()->getPos()).angle()));
    } else {
        command.set_w(static_cast<float>((ball->get()->getPos() - robot->get()->getPos()).angle()));
    }

    Vector2 velocity = robot->getControllers().getNumTreePosController()->getRobotCommand(robot->get()->getId(), targetPos).vel;
    command.mutable_vel()->set_x(static_cast<float>(velocity.x));
    command.mutable_vel()->set_y(static_cast<float>(velocity.y));
    publishRobotCommand();
    return Status::Running;
}

void ActiveStop::onTerminate(Skill::Status s) { attack = -1; }

Vector2 ActiveStop::getOffensiveActivePoint(const Field &field, const rtt::world_new::view::BallView &ball) {
    Vector2 penaltyPos = FieldComputations::getPenaltyPoint(field, false);
    return getPoint(field, ball, penaltyPos);
}

Vector2 ActiveStop::getDefensiveActivePoint(const Field &field, const rtt::world_new::view::BallView &ball) {
    Vector2 penaltyPos = FieldComputations::getPenaltyPoint(field, true);
    return getPoint(field, ball, penaltyPos);
}

Vector2 ActiveStop::getPoint(const Field &field, const rtt::world_new::view::BallView &ball, const Vector2 &penaltyPos) {
    Vector2 ballPos = ball->getPos();

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(1.2);  // ssl rule + significant buffer

    if (FieldComputations::pointIsInDefenceArea(field, ballPos + offset, true, 0.3, true)) {
        return offset;
    }
    if (FieldComputations::pointIsInDefenceArea(field, ballPos + offset, false, 0.3, true)) {
        return offset;
    }
    return ballPos + offset;
}

}  // namespace rtt::ai