//
// Created by rolf on 10/12/18.
//

#include <roboteam_ai/src/interface/drawer.h>
#include "Keeper.h"
#include "roboteam_ai/src/world/Field.h"

namespace rtt {
namespace ai {

Keeper::Keeper(rtt::string name, bt::Blackboard::Ptr blackboard)
        :Skill(std::move(name), std::move(blackboard)) { }

void Keeper::onInitialize() {

    goalPos = world::field->get_our_goal_center();
    goalwidth = world::field->get_field().goal_width;

    //Create arc for keeper to drive on
    blockCircle=control::ControlUtils::createKeeperArc();
}

Keeper::Status Keeper::onUpdate() {
    Vector2 ballPos = world::world->getBall()->pos;
    Vector2 blockPoint = computeBlockPoint(ballPos);

    if (!world::field->pointIsInField(blockPoint, static_cast<float>(Constants::OUT_OF_FIELD_MARGIN()))) {
        std::cout << "Keeper escaping field!" << std::endl;
        return Status::Running;
    }

    Vector2 velocities = gtp.getPosVelAngle(robot, blockPoint).vel;
    command.x_vel = static_cast<float>(velocities.x);
    command.y_vel = static_cast<float>(velocities.y);
    publishRobotCommand();
    return Status::Running;
}

void Keeper::onTerminate(Status s) {
    command.x_vel = 0;
    command.y_vel = 0;
    command.w = static_cast<float>(M_PI_2);
    publishRobotCommand();
}

Vector2 Keeper::computeBlockPoint(Vector2 defendPos) {
    Vector2 u1 = (goalPos + Vector2(0.0, goalwidth*0.5) - defendPos).normalize();
    Vector2 u2 = (goalPos + Vector2(0.0, - goalwidth*0.5) - defendPos).normalize();
    double dist = (defendPos - goalPos).length();
    Vector2 blockLineStart = defendPos + (u1 + u2).stretchToLength(dist);
    std::pair<boost::optional<Vector2>, boost::optional<Vector2>> intersections = blockCircle.intersectionWithLine(
            blockLineStart, defendPos);
    Vector2 blockPos, posA, posB;
    // go stand on the intersection of the lines. Pick the one that is closest to (0,0) if there are multiple
    if (intersections.first && intersections.second) {
        posA = *intersections.first;
        posB = *intersections.second;

        if (!world::field->pointIsInDefenceArea(posA, true)) {
            blockPos = posB;
        }

        if (posA.length() < posB.length()) {
            blockPos = posA;
        }
        else blockPos = posB;
    }
    else if (intersections.first) {
        blockPos = *intersections.first;
    }
    else if (intersections.second) {
        blockPos = *intersections.second;
    }
    else {
        blockPos = Vector2(goalPos.x + Constants::KEEPER_POST_MARGIN(), goalwidth/2
                *signum(defendPos.y)); // Go stand at one of the poles depending on the side the defendPos is on.
    }
    //Interface visualization:
    std::vector<std::pair<rtt::Vector2, QColor>> displayColorData;
    std::pair<rtt::Vector2, QColor> A=std::make_pair(blockPos,Qt::red);
    displayColorData.push_back(A);
    displayColorData.emplace_back(std::make_pair(blockLineStart,Qt::red));
    displayColorData.emplace_back(std::make_pair(defendPos,Qt::red));
    displayColorData.emplace_back(std::make_pair(goalPos + Vector2(0.0, goalwidth*0.5),Qt::green));
    displayColorData.emplace_back(std::make_pair(goalPos - Vector2(0.0, goalwidth*0.5),Qt::green));
    displayColorData.emplace_back(std::make_pair(robot->pos,Qt::blue));
    interface::Drawer::setKeeperPoints(robot->id,displayColorData);

    return blockPos;
}

}
}