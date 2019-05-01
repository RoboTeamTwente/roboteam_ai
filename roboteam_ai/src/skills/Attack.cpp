//
// Created by thijs on 17-12-18.
//

#include <roboteam_ai/src/control/PositionUtils.h>
#include "Attack.h"
#include <roboteam_ai/src/world/Field.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "roboteam_ai/src/coach/OffensiveCoach.h"

namespace rtt {
namespace ai {

Attack::Attack(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) {
}

void Attack::onInitialize() {
    shot = false;
    shotControl = std::make_shared<control::ShotController>();
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (! robot) return Status::Running;

    if (shot && ! world::world->ourRobotHasBall(robot->id)) {
        return Status::Success;
    }

    Vector2 aimPoint= coach::g_offensiveCoach.getShootAtGoalPoint(ball->pos);
    shotControl->makeCommand(shotControl->getShotData(robot, aimPoint), command);
    publishRobotCommand();
    return Status::Running;
}

void Attack::onTerminate(Status s) {

    command.w = static_cast<float>(deltaPos.angle());
    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand();
}

} // ai
} // rtt