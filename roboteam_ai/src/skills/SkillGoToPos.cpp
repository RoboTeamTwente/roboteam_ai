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
        :Skill(std::move(name), std::move(blackboard)) {
}

/// Called when the Skill is Initialized
void SkillGoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);
    targetPos = properties->getVector2("Position");
    goToBall = properties->getBool("goToBall");
    std::string gTT = properties->getString("goToType");
    setPosController(gTT);
}

void SkillGoToPos::setPosController(const string &gTT) {
    if (gTT == "ballControl") {
        posController = std::make_shared<control::ControlGoToPosBallControl>();
    }
    else if (gTT == "basic") {
        posController = std::make_shared<control::BasicPosControl>();
    }
    else if (gTT == "force") {
        posController = std::make_shared<control::ForcePosControl>();
    }
    else {
        ROS_ERROR("SkillGoToPos::onInitialize -> no good goToType set in properties. Using numtrees");
        posController = std::make_shared<control::NumTreePosControl>();
    }
}

/// Called when the Skill is Updated
SkillGoToPos::Status SkillGoToPos::onUpdate() {

   // goToPos.goToPos(robot, targetPos, goToType);
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
void SkillGoToPos::onTerminate(Status s) {
    command.w = 0;
    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand();
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