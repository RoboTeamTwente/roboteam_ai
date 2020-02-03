#include <control/PositionUtils.h>
#include "skills/Attack.h"
#include <world/FieldComputations.h>
#include <control/numtrees/NumTreePosControl.h>
#include <control/BasicPosControl.h>
#include <control/ControlUtils.h>
#include "coach/OffensiveCoach.h"
#include <control/PositionUtils.h>
#include <control/numtrees/NumTreePosControl.h>
#include <control/BasicPosControl.h>
#include <control/ControlUtils.h>
#include "coach/OffensiveCoach.h"

namespace rtt::ai {

    Attack::Attack(string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

/// Get an update on the skill
    bt::Node::Status Attack::onUpdate() {
        if (!robot) return Status::Running;

        if (FieldComputations::pointIsInDefenceArea(*field, ball->getPos(), false)) {
            command.set_w(robot->angle);
            publishRobotCommand();
            return Status::Running;
        }

        Vector2 aimPoint = coach::g_offensiveCoach.getShootAtGoalPoint(*field, ball->getPos());
        auto shotData = robot->getShotController()->getRobotCommand(*field, *robot, aimPoint, false,
                                                                    control::BallSpeed::MAX_SPEED, false, control::ShotPrecision::MEDIUM, 3);
        command = shotData.makeROSCommand();
        publishRobotCommand();
        return Status::Running;
    }

}  // namespace rtt::ai