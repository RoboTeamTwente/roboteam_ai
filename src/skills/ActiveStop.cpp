//
// Created by baris on 8-4-19.
//

#include <world/Field.h>
#include "skills/ActiveStop.h"
#include "control/ControlUtils.h"

namespace rtt{
namespace ai {

int ActiveStop::attack = -1;

ActiveStop::ActiveStop(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}
void ActiveStop::onInitialize() {
    robot->getNumtreePosControl()->setAvoidBallDistance(0.8);

    if (attack == -1) {
        attack = robot->id;
        attacker = true;
    }
}

Skill::Status ActiveStop::onUpdate() {
    targetPos = attacker ? getOffensiveActivePoint() : getDefensiveActivePoint();

    if (robot->pos.dist(targetPos) > 0.3) {
        command.set_w(static_cast<float>((targetPos - robot->pos).angle()));
    } else {
        command.set_w(static_cast<float>((ball->getPos() - robot->pos).angle()));
    }

    Vector2 velocity = robot->getNumtreePosControl()->getRobotCommand(world, field, robot, targetPos).vel;
    command.mutable_vel()->set_x(static_cast<float>(velocity.x));
    command.mutable_vel()->set_y(static_cast<float>(velocity.y));
    publishRobotCommand();
    return Status::Running;
}

void ActiveStop::onTerminate(Skill::Status s) {
    attack = -1;
}

Vector2 ActiveStop::getOffensiveActivePoint() {
    Vector2 penaltyPos = rtt::ai::world::field->getPenaltyPoint(false);
    return getPoint(penaltyPos);
}

Vector2 ActiveStop::getDefensiveActivePoint() {
    Vector2 penaltyPos = rtt::ai::world::field->getPenaltyPoint(true);
    return getPoint(penaltyPos);
}

Vector2 ActiveStop::getPoint(const Vector2 &penaltyPos) {
    Vector2 ballPos = world::world->getBall()->getPos();

    Vector2 offset = (penaltyPos - ballPos).stretchToLength(1.2); // ssl rule + significant buffer

    if (world::field->pointIsInDefenceArea(ballPos + offset, true, 0.3, true)) {
        return offset;
    }
    if (world::field->pointIsInDefenceArea(ballPos + offset, false, 0.3, true)) {
        return offset;
    }
    return ballPos + offset;
}

}
}