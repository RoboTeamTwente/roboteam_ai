//
// Created by rolf on 10/12/18.
//

#include <skills/Keeper.h>
#include <interface/api/Input.h>

namespace rtt::ai {

Keeper::Keeper(std::string name, bt::Blackboard::Ptr blackboard) : Skill(std::move(name), std::move(blackboard)) {}

void Keeper::onInitialize() {
    goalPos = field->getOurGoalCenter();
    goalWidth = field->getGoalWidth();
    // Create arc for keeper to drive on
    blockCircle = createKeeperArc();

    /// This function is hacky; we need to manually update the PID now
    /// everytime.
    robot->getControllers().getBasicPosController()->setAutoListenToInterface(false);
    robot->getControllers().getBasicPosController()->updatePid(Constants::standardKeeperPID());
}

Keeper::Status Keeper::onUpdate() {
    Vector2 ballPos = world->getBall()->get()->getPos();
    Vector2 blockPoint;

    goalPos = field->getOurGoalCenter();

    if (ball->get()->getPos().x < 0) {
        auto attacker = world.getRobotClosestToPoint(ball->get()->getPos(), world_new::them);
        if (attacker && (ball->get()->getPos() - attacker->getPos()).length() < MIN_ATTACKER_DIST) {
            setGoalPosWithAttacker(attacker);
        }
    }

    blockPoint = computeBlockPoint(ballPos);

    if (!FieldComputations::pointIsInField(*field, blockPoint, static_cast<float>(Constants::OUT_OF_FIELD_MARGIN()))) {
        blockPoint = goalPos;
        blockPoint.x += Constants::KEEPER_CENTREGOAL_MARGIN();
        command.set_w(0);
    } else {
        command.set_w(Angle((ballPos - blockPoint).angle() + M_PI_2).getAngle());
    }
    interface::Input::drawData(interface::Visual::KEEPER, {blockPoint}, Qt::darkYellow, robot->get()->getId(), interface::Drawing::DOTS, 5, 5);
    /// Manual PID value update. Ugly and should be refactored in the future.
    robot->getControllers().getBasicPosController()->updatePid(interface::Output::getKeeperPid());
    Vector2 velocities = robot->getControllers().getBasicPosController()->getRobotCommand(robot->get()->getId(), blockPoint).vel;
    command.mutable_vel()->set_x(velocities.x);
    command.mutable_vel()->set_y(velocities.y);
    publishRobotCommand();
    return Status::Running;
}

void Keeper::onTerminate(Status s) {}

Vector2 Keeper::computeBlockPoint(const Vector2 &defendPos) {
    Vector2 blockPos, posA, posB;
    if (defendPos.x < field->getLeftmostX()) {
        if (abs(defendPos.y) >= goalWidth) {
            blockPos = Vector2(goalPos.x + Constants::KEEPER_POST_MARGIN(), goalWidth / 2 * signum(defendPos.y));
        } else {
            blockPos = goalPos;
            blockPos.x += Constants::KEEPER_CENTREGOAL_MARGIN();
        }
    } else {
        Vector2 u1 = (goalPos + Vector2(0.0, goalWidth * 0.5) - defendPos).normalize();
        Vector2 u2 = (goalPos + Vector2(0.0, -goalWidth * 0.5) - defendPos).normalize();
        double dist = (defendPos - goalPos).length();
        Vector2 blockLineStart = defendPos + (u1 + u2).stretchToLength(dist);
        std::pair<std::optional<Vector2>, std::optional<Vector2>> intersections = blockCircle.intersectionWithLine(blockLineStart, defendPos);

        // go stand on the intersection of the lines. Pick the one that is
        // closest to (0,0) if there are multiple
        if (intersections.first && intersections.second) {
            posA = *intersections.first;
            posB = *intersections.second;

            if (!FieldComputations::pointIsInDefenceArea(*field, posA, true)) {
                blockPos = posB;
            }

            if (posA.length() < posB.length()) {
                blockPos = posA;
            } else {
                blockPos = posB;
            }
        } else if (intersections.first) {
            blockPos = *intersections.first;
        } else if (intersections.second) {
            blockPos = *intersections.second;
        } else {
            blockPos = Vector2(goalPos.x + Constants::KEEPER_POST_MARGIN(),
                               goalWidth / 2 * signum(defendPos.y));  // Go stand at one of the poles depending
            // on the side the defendPos is on.
        }
    }

    interface::Input::drawData(interface::Visual::KEEPER, {defendPos, blockPos}, Qt::red, robot->get()->getId(), interface::Drawing::DrawingMethod::DOTS, 5, 5);
    return blockPos;
}

void Keeper::setGoalPosWithAttacker(world_new::view::RobotView attacker) {
    Vector2 start;
    Vector2 end;
    double distanceToGoal = ((Vector2) attacker->getPos() - field->getOurGoalCenter()).length();

    start = attacker->getPos();

    auto goal = FieldComputations::getGoalSides(*field, true);
    Vector2 attackerToBallV2 = ball->get()->getPos() - attacker->getPos();
    Vector2 attackerAngleV2 = attacker->getAngle().toVector2();
    Vector2 i1 = control::ControlUtils::twoLineIntersection(attackerToBallV2 + attacker->getPos(), attacker->getPos(), goal.start, goal.end);
    Vector2 i2 = control::ControlUtils::twoLineIntersection(attackerAngleV2 + attacker->getPos(), attacker->getPos(), goal.start, goal.end);
    Angle targetAngle = Vector2((i1 + i2) * 0.5 - attacker->getPos()).toAngle();
    end = start + (Vector2){distanceToGoal * 1.2, 0}.rotate(targetAngle);

    if (control::ControlUtils::lineSegmentsIntersect(start, end, field->getOurBottomGoalSide(), field->getOurTopGoalSide())) {
        goalPos = control::ControlUtils::twoLineIntersection(start, end, field->getOurBottomGoalSide(),
                                                             field->getOurTopGoalSide());
    }
}

rtt::Arc Keeper::createKeeperArc() {
    double goalWidth = field->getGoalWidth();
    Vector2 goalPos = field->getOurGoalCenter();
    double diff = rtt::ai::Constants::KEEPER_POST_MARGIN() - rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN();

    double radius = diff * 0.5 + goalWidth * goalWidth / (8 * diff);  // Pythagoras' theorem.
    double angle = asin(goalWidth / 2 / radius);                      // maximum angle (at which we hit the posts)
    Vector2 center = Vector2(goalPos.x + rtt::ai::Constants::KEEPER_CENTREGOAL_MARGIN() + radius, 0);
    return diff > 0 ? rtt::Arc(center, radius, M_PI - angle, angle - M_PI) : rtt::Arc(center, radius, angle, -angle);
}

}  // namespace rtt::ai