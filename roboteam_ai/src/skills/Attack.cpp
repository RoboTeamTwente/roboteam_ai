#include <roboteam_ai/src/control/PositionUtils.h>
#include "Attack.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "roboteam_ai/src/coach/OffensiveCoach.h"
#include <roboteam_ai/src/control/PositionUtils.h>
#include "Attack.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "roboteam_ai/src/coach/OffensiveCoach.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;

    if (world::field->pointIsInDefenceArea(ball->pos, false)) {
        command.w = robot->angle;
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(ball->pos);
    auto shotData = robot->getShotController()->getRobotCommand(
            *robot, aimPoint, false, control::BallSpeed::MAX_SPEED, true, control::ShotPrecision::HIGH);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt