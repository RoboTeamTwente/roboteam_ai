//
// Created by baris on 15-1-19.
//

#include "GTPSpecial.h"

namespace rtt {
namespace ai {

GTPSpecial::GTPSpecial(string name, bt::Blackboard::Ptr blackboard)
        :GoToPos(std::move(name), std::move(blackboard)) {

}

void GTPSpecial::onInitialize() {
    if (properties->hasDouble("errorMargin")) {
        errorMargin = properties->getDouble("errorMargin");
    }

    if (properties->hasString("type")) {
        type = stringToType(properties->getString("type"));
        switch (type) {
            case goToBall: {
                targetPos = ball->pos;
                break;
            }
            case ballPlacementBefore: {
                targetPos = coach::Coach::getBallPlacementBeforePos(ball->pos);
                break;
            }
            case ballPlacementAfter: {
                errorMargin = 0.05;
                targetPos = coach::Coach::getBallPlacementAfterPos(robot->angle);
                break;
            }
            case getBallFromSide: {
                targetPos = getBallFromSideLocation();
                break;
            }
        }
    }

    if (properties->hasDouble("maxVel")) {
        maxVel = properties->getDouble("maxVel");
    }

    goToPos.setAvoidBall(properties->getBool("avoidBall"));
    goToPos.setCanGoOutsideField(properties->getBool("canGoOutsideField"));
}

Vector2 GTPSpecial::getBallFromSideLocation() {
    roboteam_msgs::GeometryFieldSize field = Field::get_field();
    double distanceFromTop      = abs(field.field_width / 2 - ball->pos.y);
    double distanceFromBottom   = abs(-field.field_width / 2 - ball->pos.y);
    double distanceFromLeft     = abs(-field.field_length / 2 - ball->pos.x);
    double distanceFromRight    = abs(field.field_length / 2 - ball->pos.x);

    double distance = INT_MAX;
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

GTPSpecial::Type GTPSpecial::stringToType(std::string string) {
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
}

}
}
