//
// Created by thijs on 19-11-18.
//

#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/control/positionControllers/ForcePosControl.h>
#include <roboteam_ai/src/control/positionControllers/BasicPosControl.h>
#include <roboteam_ai/src/control/positionControllers/ControlGoToPosBallControl.h>
#include "SkillGoToPos.h"

namespace rtt {
namespace ai {

/// GoToPosLuTh: obstacle avoidance following Lukas & Thijs principles
SkillGoToPos::SkillGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(std::move(name), std::move(blackboard)) {
}

/// Called when the Skill is Initialized
void SkillGoToPos::gtpInitialize() {
}

/// Called when the Skill is Updated
SkillGoToPos::Status SkillGoToPos::gtpUpdate() {

    if (goToBall) targetPos = ball->pos;

    // Now check the progress we made
    currentProgress = checkProgression();
    // Send a move command

    switch (currentProgress) {
        // Return the progression in terms of status
        case ON_THE_WAY:
            return Status::Running;
        case DONE:
            return Status::Success;
        case FAIL:
            return Status::Failure;
    }

    return Status::Failure;
}

/// Called when the Skill is Terminated
void SkillGoToPos::gtpTerminate(Status s) {
}

SkillGoToPos::Progression SkillGoToPos::checkProgression() {

    double dx = targetPos.x - robot->pos.x;
    double dy = targetPos.y - robot->pos.y;
    Vector2 deltaPos = {dx, dy};

    double maxMargin = 0.3;                        // max offset or something.

    if (deltaPos.length() >= maxMargin) return ON_THE_WAY;
    else return DONE;
}

} // ai
} // rtt