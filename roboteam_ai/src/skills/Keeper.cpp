//
// Created by rolf on 10/12/18.
//

#include <roboteam_ai/src/interface/api/Input.h>
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
    Vector2 blockPoint;

    goalPos = world::field->get_our_goal_center();

    if (ball->pos.x < 0) {
        auto attacker = world::world->getRobotClosestToPoint(ball->pos, world::THEIR_ROBOTS);
        if ((ball->pos - attacker.pos).length() < 0.5) {
            setGoalPosWithAttacker(attacker);
        }
    }

    blockPoint = computeBlockPoint(ballPos);

    if (!world::field->pointIsInField(blockPoint, static_cast<float>(Constants::OUT_OF_FIELD_MARGIN()))) {
        return Status::Running;
    }

    Vector2 velocities = basicGtp.getPosVelAngle(robot, blockPoint).vel;
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

    interface::Input::drawData(interface::Visual::KEEPER, {defendPos, blockPos}, Qt::red, robot->id, interface::Drawing::DrawingMethod::DOTS, 5, 5);
    return blockPos;
}

void Keeper::setGoalPosWithAttacker(world::Robot attacker) {
    Vector2 start;
    Vector2 end;
    double distanceToGoal = ((Vector2)attacker.pos - world::field->get_our_goal_center()).length();

    start = attacker.pos;

    auto goal = world::field->getGoalSides(false);
    Vector2 attackerToBallV2 = ball->pos - attacker.pos;
    Vector2 attackerAngleV2 = attacker.angle.toVector2();
    Vector2 i1 = control::ControlUtils::twoLineIntersection(attackerToBallV2 + attacker.pos, attacker.pos, goal.first, goal.second);
    Vector2 i2 = control::ControlUtils::twoLineIntersection(attackerAngleV2 + attacker.pos, attacker.pos, goal.first, goal.second);
    Angle targetAngle = Vector2(attacker.pos - (i1 + i2)*0.5).toAngle();
    end = start + (Vector2){distanceToGoal * 1.2, 0}.rotate(targetAngle);

    auto field = world::field->get_field();
    Vector2 startGoal = {-field.field_length / 2, -field.goal_width / 2};
    Vector2 endGoal = {-field.field_length / 2, field.goal_width / 2};
    if (control::ControlUtils::lineSegmentsIntersect(start, end, startGoal, endGoal)) {
        goalPos = control::ControlUtils::twoLineIntersection(start, end, startGoal, endGoal);
    }
}

}
}