#include <control/PositionUtils.h>
#include "skills/Attack.h"
#include <world/Field.h>
#include <control/numTrees/NumTreePosControl.h>
#include <control/BasicPosControl.h>
#include <control/ControlUtils.h>
#include "coach/OffensiveCoach.h"
#include <control/PositionUtils.h>
#include "skills/Attack.h"
#include <world/Field.h>
#include <control/numTrees/NumTreePosControl.h>
#include <control/BasicPosControl.h>
#include <control/ControlUtils.h>
#include "coach/OffensiveCoach.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}
Attack::Attack(string name, bt::Blackboard::Ptr blackboard, rtt::ai::coach::OffensiveCoach offensiveCoach)
        : Skill(std::move(name), std::move(blackboard)) {
    this->offensiveCoach = offensiveCoach;
}

rtt::ai::coach::OffensiveCoach Attack::getOffensiveCoach() {
    return this->offensiveCoach;
}
/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;

    if (field->pointIsInDefenceArea(ball->getPos(), false)) {
        command.set_w(robot->angle);
        publishRobotCommand();
        return Status::Running;
    }

    Vector2 aimPoint = this->getOffensiveCoach().getShootAtGoalPoint(ball->getPos());
    auto shotData = robot->getShotController()->getRobotCommand(
            *robot, aimPoint, false, control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::MEDIUM,3);
    command = shotData.makeROSCommand();
    publishRobotCommand();
    return Status::Running;
}

} // ai
} // rtt