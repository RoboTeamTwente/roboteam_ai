//
// Created by thijs on 19-11-18.
//

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
    if (gTT.empty()) {
        ROS_ERROR("SkillGoToPos::onInitialize -> no goToType set in properties");
        goToType = control::PosControlType::NO_PREFERENCE;
    }
    else if (gTT == "noPreference") goToType = control::PosControlType::NO_PREFERENCE;
    else if (gTT == "ballControl") goToType = control::PosControlType::BALL_CONTROL;
    else if (gTT == "basic") goToType = control::PosControlType::BASIC;
    else if (gTT == "force") goToType = control::PosControlType::FORCE;
    else {
        ROS_ERROR("SkillGoToPos::onInitialize -> no good goToType set in properties");
        goToType = control::PosControlType::NO_PREFERENCE;
    }
}

/// Called when the Skill is Updated
SkillGoToPos::Status SkillGoToPos::onUpdate() {

    control::PositionController goToPos;

    goToPos.goToPos(robot, targetPos, goToType);
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