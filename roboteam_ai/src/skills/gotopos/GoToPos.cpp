//
// Created by baris on 24/10/18.
//

#include "GoToPos.h"
#include "roboteam_ai/src/world/Field.h"
#include "roboteam_ai/src/coach/GeneralPositionCoach.h"
#include "roboteam_ai/src/control/ControlUtils.h"

namespace rtt {
namespace ai {

GoToPos::GoToPos(string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

GoToPos::GoToType GoToPos::stringToGoToType(const string &gtt) {
    if (gtt == "ballControl") return ballControl;
    if (gtt == "basic") return basic;
    if (gtt == "force") return force;
    if (gtt == "numTrees") return numTree;

    ROS_ERROR("SkillGoToPos::onInitialize -> no good goToType set in properties. Using numtrees");
    return numTree;
}

void GoToPos::onInitialize() {
    targetPos = properties->getVector2("Position");
    if (properties->hasDouble("maxVel")) {
        maxVel = properties->getDouble("maxVel");
    }
    else {
        maxVel = Constants::DEFAULT_MAX_VEL();
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
    posController->setAvoidBall(avoidBallDistance);
    posController->setCanMoveOutOfField(properties->getBool("canGoOutsideField"));
    posController->setCanMoveInDefenseArea(properties->getBool("canMoveInDefenseArea"));
}

void GoToPos::setPositionController(const GoToType &gTT) {
    switch (gTT) {
    default:
    case numTree:posController = std::make_shared<control::NumTreePosControl>();
        return;
    case ballControl:posController = std::make_shared<control::ControlGoToPosBallControl>();
        return;
    case basic:posController = std::make_shared<control::BasicPosControl>();
        return;
    case force:posController = std::make_shared<control::ForcePosControl>();
        return;
    }
}

/// Get an update on the skill
bt::Node::Status GoToPos::onUpdate() {

    Status gtpStatus = gtpUpdate();
    switch (gtpStatus) {
    default:
    case Status::Failure:
    case Status::Success:
    case Status::Waiting:return gtpStatus;
    case Status::Running:break;
    }

    if ((targetPos - robot->pos).length2() < errorMargin*errorMargin) {
        return Status::Success;
    }

    control::PosVelAngle pva = posController->getPosVelAngle(robot, targetPos);
    pva.vel = control::ControlUtils::velocityLimiter(pva.vel, maxVel);
    command.x_vel = static_cast<float>(pva.vel.x);
    command.y_vel = static_cast<float>(pva.vel.y);
    command.w = static_cast<float>(pva.angle);

    publishRobotCommand();
    return Status::Running;
}

void GoToPos::onTerminate(Status s) {
    command.w = robot->angle;
    command.x_vel = 0;
    command.y_vel = 0;

    publishRobotCommand();
}

} // ai
} // rtt