//
// Created by thijs on 19-11-18.
//

#include "SkillGoToPos.h"

namespace rtt {
namespace ai {

/// GoToPosLuTh: obstacle avoidance following Lukas & Thijs principles
SkillGoToPos::SkillGoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {
}

/// Called when the Skill is Initialized
void SkillGoToPos::onInitialize() {
    robot = getRobotFromProperties(properties);

    targetPos = properties->getVector2("Position");
    goToBall = properties->getBool("goToBall");

    std::string gTT = properties->getString("goToType");
    if (gTT.empty()) {
        ROS_ERROR("SkillGoToPos::onInitialize -> no goToType set in properties");
        goToType = GoToType::noPreference;
    }
    else if (gTT == "noPreference") goToType = GoToType::noPreference;
    else if (gTT == "ballControl") goToType = GoToType::ballControl;
    else if (gTT == "basic") goToType = GoToType::basic;
    else if (gTT == "lowLevel") goToType = GoToType::lowLevel;
    else if (gTT == "highLevel") goToType = GoToType::highLevel;
    else if (gTT == "force") goToType = GoToType::force;
    else if (gTT == "luTh") goToType = GoToType::luTh;
    else if (gTT == "bezier") goToType = GoToType::bezier;
    else {
        ROS_ERROR("SkillGoToPos::onInitialize -> no good goToType set in properties");
        goToType = GoToType::noPreference;
    }
}

/// Called when the Skill is Updated
SkillGoToPos::Status SkillGoToPos::onUpdate() {

    control::ControlGoToPos goToPos;

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
    roboteam_msgs::RobotCommand command;
    command.id = robot->id;
    command.use_angle = 0;
    command.w = 0;

    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand(command);
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