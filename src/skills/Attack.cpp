#include <skills/Attack.h>
#include <world_new/FieldComputations.hpp>
#include "coach/OffensiveCoach.h"

namespace rtt::ai {

Attack::Attack(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (!robot) return Status::Running;

    if (world_new::FieldComputations::pointIsInDefenceArea(*field, ball->get()->getPos(), false)) {
        command.set_w(robot->get()->getAngle());
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(*field, ball->get()->getPos());
    auto shotData =
            robot->getControllers().getShotController()->getRobotCommand(*field, *robot, aimPoint, false,
                                                                         control::BallSpeed::MAX_SPEED,
                                                                         control::ShotPrecision::MEDIUM,
                                                                         <#initializer#>, <#initializer#>);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

}  // namespace rtt::ai