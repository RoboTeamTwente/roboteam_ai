//
// Created by baris on 15-1-19.
//

#include <coach/BallplacementCoach.h>
#include <skills/gotopos/GTPSpecial.h>
#include <world_new/FieldComputations.hpp>

namespace rtt::ai {
/**
 * Creates a movement path for the robot to a certain place. The "type" set in the blackboard given to this function is very important, see the update function.
 * Before using this class, it is nice to have a look inside, as its behaviour is not
 * @param name the name for the node (shows up in the tree visualiser widget (probably))
 * @param blackboard the blackboard passed into the GTPSpecial, for example, to set the "type" of the GTPSpecial
 */
GTPSpecial::GTPSpecial(std::string name, bt::Blackboard::Ptr blackboard) : GoToPos(std::move(name), std::move(blackboard)) {}

void GTPSpecial::gtpInitialize() {
    type = stringToType(properties->getString("type"));
    goToType = stringToGoToType(properties->getString("goToType"));
    switch (type) {
        case goToBall: {
            maxVel = 9e9;
            targetPos = ball->get()->getPos();

            if (goToType == basic) {
                robot->getControllers().getBasicPosController()->setAvoidBallDistance(false);
            }

            else if (goToType == numTree) {
                robot->getControllers().getNumTreePosController()->setAvoidBallDistance(false);
            }
            break;
        }
        case ballPlacementBefore: {
            maxVel = 1.0;
            targetPos = coach::g_ballPlacement.getBallPlacementBeforePos(ball->get()->getPos());
            break;
        }
        case ballPlacementAfter: {
            targetPos = coach::g_ballPlacement.getBallPlacementAfterPos(*robot);
            break;
        }
        case getBallFromSide: {
            maxVel = 9e9;
            targetPos = getBallFromSideLocation();
            break;
        }
        case defaultType: {
            maxVel = 9e9;
            targetPos = {0, 0};
            std::cout << "GTPSpecial was not given any type! Defaulting to {0, 0}" << std::endl;
            break;
        }
        case freeKick: {
            maxVel = 9e9;
            Vector2 ballPos = rtt::ai::world::world->getBall()->getPos();
            Vector2 penaltyThem = world_new::FieldComputations::getPenaltyPoint(*field, false);
            targetPos = (ballPos + (penaltyThem - ballPos).stretchToLength((penaltyThem - ballPos).length() / 2.0));
            errorMargin = 0.05;
            break;
        }
        case getBackIn: {
            if (goToType == basic) {
                robot->getControllers().getBasicPosController()->setCanMoveInDefenseArea(true);
            }

            else if (goToType == numTree) {
                robot->getControllers().getNumTreePosController()->setCanMoveInDefenseArea(true);
            }

            targetPos = {0, 0};
            break;
        }
        case ourGoalCenter: {
            targetPos = (*field).getOurGoalCenter();
            break;
        }
        case ourDefenseAreaCenter: {
            targetPos = world_new::FieldComputations::getDefenseArea(*field).centroid();
            break;
        }
    }
}

Vector2 GTPSpecial::getBallFromSideLocation() {
    double distanceFromTop = abs((*field).getFieldWidth() * 0.5 - ball->get()->getPos().y);
    double distanceFromBottom = abs(-(*field).getFieldWidth() * 0.5 - ball->get()->getPos().y);
    double distanceFromLeft = abs(-(*field).getFieldLength() * 0.5 - ball->get()->getPos().x);
    double distanceFromRight = abs((*field).getFieldLength() * 0.5 - ball->get()->getPos().x);

    double distance = 9e9;
    Vector2 pos;
    if (distanceFromTop < distance) {
        distance = distanceFromTop;
        pos = {ball->get()->getPos().x, ball->get()->getPos().y - getballFromSideMargin};
    }
    if (distanceFromBottom < distance) {
        distance = distanceFromBottom;
        pos = {ball->get()->getPos().x, ball->get()->getPos().y + getballFromSideMargin};
    }

    if (distance < 0.20) {
        return pos;
    }
    if (distanceFromLeft < distance) {
        distance = distanceFromLeft;
        pos = {ball->get()->getPos().x + getballFromSideMargin, ball->get()->getPos().y};
    }
    if (distanceFromRight < distance) {
        pos = {ball->get()->getPos().x - getballFromSideMargin, ball->get()->getPos().y};
    }

    return pos;
}

GTPSpecial::Type GTPSpecial::stringToType(const std::string &string) {
    if (string == "goToBall") {
        return goToBall;
    } else if (string == "ballPlacementBefore") {
        return ballPlacementBefore;
    } else if (string == "ballPlacementAfter") {
        return ballPlacementAfter;
    } else if (string == "getBallFromSide") {
        return getBallFromSide;
    } else if (string == "freeKick") {
        return freeKick;
    } else if (string == "getBackIn") {
        return getBackIn;
    } else if (string == "ourGoalCenter") {
        return ourGoalCenter;
    } else if (string == "ourDefenseAreaCenter") {
        return ourDefenseAreaCenter;
    } else {
        return defaultType;
    }
}

Skill::Status GTPSpecial::gtpUpdate() {
    switch (type) {
        default:
            break;
        case goToBall: {
            maxVel = 9e9;
            targetPos = ball->get()->getPos();
            break;
        }
        case getBackIn: {
            targetPos = {0, 0};
            break;
        }
        case ballPlacementBefore:
            maxVel = 1.0;
            break;
        case ourGoalCenter: {
            targetPos = (*field).getOurGoalCenter();
            robot->getControllers().getNumTreePosController()->setCanMoveInDefenseArea(true);
            command = robot->getControllers().getNumTreePosController()->getRobotCommand(world, field, *robot, targetPos, true).makeROSCommand();
            break;
        }
        case ourDefenseAreaCenter: {
            targetPos = world_new::FieldComputations::getDefenseArea(*field).centroid();
            robot->getControllers().getNumTreePosController()->setCanMoveInDefenseArea(true);
            command = robot->getControllers().getNumTreePosController()->getRobotCommand(world, field, *robot, targetPos, true).makeROSCommand();
            break;
        }
        case ballPlacementAfter: {
            targetPos = coach::g_ballPlacement.getBallPlacementAfterPos(*robot);
            auto c = robot->getControllers().getNumTreePosController()->getRobotCommand(world, field, *robot, targetPos);
            command = c.makeROSCommand();
            command.set_w((ball->get()->getPos() - robot->get()->getPos()).toAngle());
            maxVel = 2.0;
            break;
        }
        case getBallFromSide:
            maxVel = 9e9;
            break;
        case defaultType:
            maxVel = 9e9;
            break;
        case freeKick: {
            maxVel = 9e9;
            command.set_w((ball->get()->getPos() - robot->get()->getPos()).angle());
        }
    }

    return Status::Running;
}

void GTPSpecial::gtpTerminate(Status s) {}

}  // namespace rtt::ai
