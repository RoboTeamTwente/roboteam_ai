//
// Created by thijs on 19-11-18.
//

#include "skills/gotopos/SkillGoToPos.h"

#include <control/BasicPosControl.h>
#include <control/ball-handling/BallHandlePosControl.h>
#include <control/numtrees/NumTreePosControl.h>

namespace rtt::ai {

/// GoToPosLuTh: obstacle avoidance following Lukas & Thijs principles
SkillGoToPos::SkillGoToPos(std::string name, bt::Blackboard::Ptr blackboard) : GoToPos(std::move(name), std::move(blackboard)) {}

/// Called when the Skill is Initialized
void SkillGoToPos::gtpInitialize() {
    goToBall = properties->getBool("goToBall");
    currentProgress = ON_THE_WAY;
}

/// Called when the Skill is Updated
SkillGoToPos::Status SkillGoToPos::gtpUpdate() {
    if (goToBall) targetPos = ball->getPos();

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
void SkillGoToPos::gtpTerminate(Status s) { currentProgress = ON_THE_WAY; }

SkillGoToPos::Progression SkillGoToPos::checkProgression() {
    if ((targetPos - robot->pos).length2() > errorMargin * errorMargin)
        return ON_THE_WAY;
    else
        return DONE;
}

}  // namespace rtt::ai