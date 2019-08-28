#include <include/roboteam_ai/control/PositionUtils.h>
#include "include/roboteam_ai/skills/Attack.h"
#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/control/numTrees/NumTreePosControl.h>
#include <include/roboteam_ai/control/BasicPosControl.h>
#include <include/roboteam_ai/control/ControlUtils.h>
#include "include/roboteam_ai/coach/OffensiveCoach.h"
#include <include/roboteam_ai/control/PositionUtils.h>
#include "include/roboteam_ai/skills/Attack.h"
#include <include/roboteam_ai/world/Field.h>
#include <include/roboteam_ai/control/numTrees/NumTreePosControl.h>
#include <include/roboteam_ai/control/BasicPosControl.h>
#include <include/roboteam_ai/control/ControlUtils.h>
#include "include/roboteam_ai/coach/OffensiveCoach.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;

    if (world::field->pointIsInDefenceArea(ball->pos, false)) {
        command.set_w(robot->angle);
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(ball->pos);
    auto shotData = robot->getShotController()->getRobotCommand(
            *robot, aimPoint, false, control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::MEDIUM,3);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt