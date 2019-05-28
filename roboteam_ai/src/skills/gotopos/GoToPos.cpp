//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"
#include "roboteam_ai/src/world/Field.h"
#include "roboteam_ai/src/control/ControlUtils.h"

namespace rtt {
namespace ai {

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

GoToPos::GoToType GoToPos::stringToGoToType(const string &gtt) {
    if (gtt == "basic") return basic;
    if (gtt == "force") return force;
    if (gtt == "numTrees") return numTree;

    ROS_ERROR("SkillGoToPos::onInitialize -> no good goToType set in properties. Using numtrees");
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
    case force:std::cout << "force pos controller is deprecated " << std::endl;
        posController = std::make_shared<control::ForcePosControl>();
        return;
    }
}

/// Get an update on the skill
bt::Node::Status GoToPos::onUpdate() {
    //reset velocity and angle commands
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = 0;

    Status gtpStatus = gtpUpdate();
    if (gtpStatus != Status::Running) return gtpStatus;

    // check targetPos
    if ((targetPos - robot->pos).length2() < errorMargin*errorMargin) {
        // check targetAngle
        if (targetAngle == 0.0 || abs(targetAngle - robot->angle) < angleErrorMargin) {
            return Status::Success;
        }
    }
    if (command.x_vel == 0 || command.y_vel == 0 || command.w == 0) {
    control::PosVelAngle pva = posController->getPosVelAngle(robot, targetPos, targetAngle);

        // set robotcommands if they have not been set yet in gtpUpdate()
        command.x_vel = command.x_vel == 0 ? pva.vel.x : command.x_vel;
        command.y_vel = command.y_vel == 0 ? pva.vel.y : command.y_vel;
        command.w = command.w == 0 ? static_cast<float>(pva.angle) : command.w;
    }

    publishRobotCommand();
    return Status::Running;
}

void GoToPos::onTerminate(Status s) {
    gtpTerminate(s);
}

} // ai
} // rtt