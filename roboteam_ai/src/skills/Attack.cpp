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

///TODO: USED TO TEST THE NEW ROBOT, DO NOT USE!!!!!
///TODO: USED TO TEST THE NEW ROBOT, DO NOT USE!!!!!
///TODO: USED TO TEST THE NEW ROBOT, DO NOT USE!!!!!
///TODO: USED TO TEST THE NEW ROBOT, DO NOT USE!!!!!

void Attack::onInitialize() {
    shotControl = std::make_shared<control::ShotController>(control::ShotPrecision::LOW, control::BallSpeed::MAX_SPEED, false);
}

/// Get an update on the skill
bt::Node::Status Attack::onUpdate() {
    if (!robot) return Status::Running;

    Vector2 aimPoint= coach::g_offensiveCoach.getShootAtGoalPoint(ball->pos);
    shotControl->makeCommand(shotControl->getShotData(*robot, aimPoint), command);
    publishRobotCommand();
    return Status::Running;
}

void Attack::onTerminate(Status s) {

}

} // ai
} // rtt