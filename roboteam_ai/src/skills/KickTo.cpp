#include <roboteam_ai/src/control/PositionUtils.h>
#include "KickTo.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/numTrees/NumTreePosControl.h>
#include <roboteam_ai/src/control/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>

namespace rtt {
namespace ai {

KickTo::KickTo(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}
void KickTo::onInitialize() {
    shootPos=properties->getVector2("shootPos");
}
/// Get an update on the skill
bt::Node::Status KickTo::onUpdate() {
    if (! robot) return Status::Running;

    if (world::field->pointIsInDefenceArea(ball->pos, false)) {
        command.w = 0;
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = shootPos;
    auto shotData = robot->getShotController()->getRobotCommand(
            *robot, aimPoint, false, control::BallSpeed::LAY_STILL_AT_POSITION, true, control::ShotPrecision::HIGH);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt