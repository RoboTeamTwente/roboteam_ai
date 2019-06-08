//
// Created by baris on 15-1-19.
//

#include "GTPSpecial.h"

namespace rtt {
namespace ai {

GTPSpecial::GTPSpecial(string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(std::move(name), std::move(blackboard)) {
}

void GTPSpecial::gtpInitialize() {

    type = stringToType(properties->getString("type"));
    switch (type) {
    case goToBall: {
        maxVel = 9e9;
        targetPos = ball->pos;
        posController->setAvoidBallDistance(false);
        break;
    }
    case ballPlacementBefore: {
        maxVel = 1.0;
        targetPos = coach::g_ballPlacement.getBallPlacementBeforePos(ball->pos);
        break;
    }
    case ballPlacementAfter: {
        targetPos = coach::g_ballPlacement.getBallPlacementAfterPos(robot);
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
        ROS_WARN("GTPSpecial was not given any type! Defaulting to {0, 0}");
        break;
    }
    case freeKick: {
        maxVel = 9e9;
        Vector2 ballPos = rtt::ai::world::world->getBall()->pos;

        Vector2 penaltyThem = rtt::ai::world::field->getPenaltyPoint(false);
        targetPos = (ballPos + (penaltyThem - ballPos).stretchToLength((penaltyThem - ballPos).length()/2.0));
        errorMargin = 0.05;
        break;
    }
    case getBackIn: {
        posController->setCanMoveInDefenseArea(true);

        targetPos = {0, 0};
        break;
    }
    case ourGoalCenter: {
        targetPos = world::field->get_our_goal_center();
        break;
    }
    case ourDefenseAreaCenter: {
        targetPos = world::field->getDefenseArea().centroid();
        break;
    }
    case theirGoalCenter: {
        targetPos = world::field->get_their_goal_center();
        break;
    }
    }

}

Vector2 GTPSpecial::getBallFromSideLocation() {
    roboteam_msgs::GeometryFieldSize field = world::field->get_field();
    double distanceFromTop = abs(field.field_width*0.5 - ball->pos.y);
    double distanceFromBottom = abs(- field.field_width*0.5 - ball->pos.y);
    double distanceFromLeft = abs(- field.field_length*0.5 - ball->pos.x);
    double distanceFromRight = abs(field.field_length*0.5 - ball->pos.x);

    double distance = 9e9;
    Vector2 pos;
    if (distanceFromTop < distance) {
        distance = distanceFromTop;
        pos = {ball->pos.x, ball->pos.y - getballFromSideMargin};
    }
    if (distanceFromBottom < distance) {
        distance = distanceFromBottom;
        pos = {ball->pos.x, ball->pos.y + getballFromSideMargin};
    }

    if (distance < 0.20) {
        return pos;
    }
    if (distanceFromLeft < distance) {
        distance = distanceFromLeft;
        pos = {ball->pos.x + getballFromSideMargin, ball->pos.y};
    }
    if (distanceFromRight < distance) {
        pos = {ball->pos.x - getballFromSideMargin, ball->pos.y};
    }

    return pos;
}

GTPSpecial::Type GTPSpecial::stringToType(const std::string &string) {
    if (string == "goToBall") {
        return goToBall;
    }
    else if (string == "ballPlacementBefore") {
        return ballPlacementBefore;
    }
    else if (string == "ballPlacementAfter") {
        return ballPlacementAfter;
    }
    else if (string == "getBallFromSide") {
        return getBallFromSide;
    }
    else if (string == "freeKick") {
        return freeKick;
    }
    else if (string == "getBackIn") {
        return getBackIn;
    }
    else if (string == "ourGoalCenter") {
        return ourGoalCenter;
    }
    else if (string == "theirGoalCenter") {
        return theirGoalCenter;
    }
    else if (string == "ourDefenseAreaCenter") {
        return ourDefenseAreaCenter;
    }
    else {
        return defaultType;
    }
}

Skill::Status GTPSpecial::gtpUpdate() {
    switch (type) {
    default:break;
    case goToBall: {
        maxVel = 9e9;
        targetPos = ball->pos;
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
        targetPos = world::field->get_our_goal_center();
        robot->getNumtreePosControl()->setCanMoveInDefenseArea(true);
        command = robot->getNumtreePosControl()->getRobotCommand(robot, targetPos, true).makeROSCommand();
        break;
    }
    case ourDefenseAreaCenter: {
        targetPos = rtt::ai::world::field->getDefenseArea().centroid();
        robot->getNumtreePosControl()->setCanMoveInDefenseArea(true);
        command = robot->getNumtreePosControl()->getRobotCommand(robot, targetPos, true).makeROSCommand();
        break;
    }
    case ballPlacementAfter:{
        targetPos = coach::g_ballPlacement.getBallPlacementAfterPos(robot);
            auto c = robot->getBasicPosControl()->getRobotCommand(robot, targetPos);
            command = c.makeROSCommand();
            command.w = (ball->pos - robot->pos).toAngle();
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
        command.w = (ball->pos - robot->pos).angle();
    }
    }

    return Status::Running;
}

void GTPSpecial::gtpTerminate(Status s) {
}

}
}
