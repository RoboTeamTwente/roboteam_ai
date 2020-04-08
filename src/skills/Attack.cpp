#include <skills/Attack.h>
#include "coach/OffensiveCoach.h"

namespace rtt::ai {

Attack::Attack(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (!robot) return Status::Running;

    if (FieldComputations::pointIsInDefenseArea(*field, ball->get()->getPos(), false)) {
        command.set_w(robot->get()->getAngle());
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(*field, ball->get()->getPos());
    auto shotData =
        robot->getControllers().getShotController()->getRobotCommand(robot->get()->getId(), aimPoint, false, control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::MEDIUM, 3);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

}  // namespace rtt::ai