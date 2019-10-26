//
// Created by baris on 24/10/18.
//

#include "skills/gotopos/GoToPos.h"
#include "world/Field.h"
#include "control/ControlUtils.h"

namespace rtt {
namespace ai {

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

GoToPos::GoToType GoToPos::stringToGoToType(const string &gtt) {
    if (gtt == "basic") return basic;
    if (gtt == "numTrees") return numTree;

    std::cerr << "SkillGoToPos::onInitialize -> no good goToType set in properties. Using numtrees" << std::endl;
    return numTree;
}

void GoToPos::onInitialize() {
    targetPos = properties->getVector2("targetPos");
    if (properties->hasDouble("errorMargin")) {
        errorMargin = properties->getDouble("errorMargin");
    }
    if (properties->hasDouble("maxVel")) {
        maxVel = properties->getDouble("maxVel");
    }
    else {
        maxVel = Constants::MAX_VEL();
    }

    goToType = stringToGoToType(properties->getString("goToType"));
    setPositionController(goToType);

    double avoidBallDistance = 0.0;
    if (properties->hasDouble("avoidBall")) {
        avoidBallDistance = properties->getDouble("avoidBall");
    }
    else if (properties->hasBool("avoidBall")) {
        avoidBallDistance = Constants::DEFAULT_BALLCOLLISION_RADIUS();
    }
    posController->setAvoidBallDistance(avoidBallDistance);
    posController->setCanMoveOutOfField(properties->getBool("canGoOutsideField"));
    posController->setCanMoveInDefenseArea(properties->getBool("canMoveInDefenseArea"));

    gtpInitialize();
}

void GoToPos::setPositionController(const GoToType &gTT) {
    switch (gTT) {
    default:
    case numTree:posController = robot->getNumtreePosControl();
        return;
    case basic:posController = robot->getBasicPosControl();
        return;
    }
}

/// Get an update on the skill
bt::Node::Status GoToPos::onUpdate() {
    //reset velocity and angle commands
    command.mutable_vel()->set_x(0);
    command.mutable_vel()->set_y(0);
    command.set_w(0);

    Status gtpStatus = gtpUpdate();
    if (gtpStatus != Status::Running) return gtpStatus;

    // check targetPos
    if ((targetPos - robot->pos).length2() < errorMargin*errorMargin) {
        // check targetAngle
        if (targetAngle == 0.0 || abs(targetAngle - robot->angle) < angleErrorMargin) {
            return Status::Success;
        }
    }
    if (command.vel().x() == 0 || command.vel().y() == 0 || command.w() == 0) {
    auto robotCommand = posController->getRobotCommand(robot, targetPos, targetAngle);

        // set robotcommands if they have not been set yet in gtpUpdate()

        command.mutable_vel()->set_x(command.vel().x() == 0 ? static_cast<float>(robotCommand.vel.x) : command.vel().x());
        command.mutable_vel()->set_y(command.vel().y() == 0 ? static_cast<float>(robotCommand.vel.y) : command.vel().y());
        command.set_w(command.w() == 0 ? static_cast<float>(robotCommand.angle) : command.w());
    }

    publishRobotCommand();
    return Status::Running;
}

void GoToPos::onTerminate(Status s) {
    gtpTerminate(s);
}


} // ai
} // rtt