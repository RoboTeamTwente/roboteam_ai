#include "skills/Attack.h"
#include <control/BasicPosControl.h>
#include <control/ControlUtils.h>
#include <control/PositionUtils.h>
#include <control/numTrees/NumTreePosControl.h>
#include <world/Field.h>
#include "coach/OffensiveCoach.h"
#include "skills/Attack.h"

namespace rtt::ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (!robot) return Status::Running;

    if (field->pointIsInDefenceArea(ball->getPos(), false)) {
        command.set_w(robot->angle);
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(ball->getPos());
    auto shotData = robot->getShotController()->getRobotCommand(*robot, aimPoint, false, control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::MEDIUM, 3);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

}  // namespace rtt::ai