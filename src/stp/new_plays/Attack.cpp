//
// Created by jordi on 24-03-20.
//

#include "stp/new_plays/Attack.h"

#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/GoalVisionFromBallInvariant.h"
#include "stp/invariants/WeHaveBallInvariant.h"
#include "stp/invariants/game_states/NormalOrFreeKickUsGameStateInvariant.h"
#include "stp/new_roles/Attacker.h"
#include "stp/new_roles/Defender.h"
#include "stp/new_roles/Formation.h"
#include "stp/new_roles/Keeper.h"

namespace rtt::ai::stp::play {

Attack::Attack() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalOrFreeKickUsGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::WeHaveBallInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::GoalVisionFromBallInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalOrFreeKickUsGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                 std::make_unique<role::Attacker>(role::Attacker("attacker")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("offender_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_3")),
                                                                                 std::make_unique<role::Formation>(role::Formation("midfielder_4")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_1")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_2")),
                                                                                 std::make_unique<role::Defender>(role::Defender("defender_3"))};
}

uint8_t Attack::score(world_new::World *world) noexcept { return 80; }

Dealer::FlagMap Attack::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag attackerFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"attacker", {attackerFlag}});
    flagMap.insert({"offender_1", {closeToTheirGoalFlag}});
    flagMap.insert({"offender_2", {closeToTheirGoalFlag}});
    flagMap.insert({"midfielder_1", {not_important}});
    flagMap.insert({"midfielder_2", {not_important}});
    flagMap.insert({"midfielder_3", {not_important}});
    flagMap.insert({"midfielder_4", {not_important}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    flagMap.insert({"defender_3", {closeToOurGoalFlag}});

    return flagMap;
}

void Attack::calculateInfoForRoles() noexcept {
    // Keeper
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());

    // TODO: Improve roles
    // Attacker
    auto goalTarget = calculateGoalTarget();
    stpInfos["attacker"].setPositionToShootAt(goalTarget);
    stpInfos["attacker"].setKickChipType(MAX);

    // Offenders

    stpInfos["offender_1"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, field.getFieldWidth() / 4));

    stpInfos["offender_2"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, -field.getFieldWidth() / 4));

    // Midfielders
    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, field.getFieldWidth() / 4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(0.0, -field.getFieldWidth() / 4));
    stpInfos["midfielder_3"].setPositionToMoveTo(Vector2(field.getFieldLength() / 8, 0.0));
    stpInfos["midfielder_4"].setPositionToMoveTo(Vector2(-field.getFieldLength() / 8, 0.0));

    // Defenders
    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world_new::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurTopGoalSide(), world_new::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_3"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_3"].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurBottomGoalSide(), world_new::them));
    stpInfos["defender_3"].setBlockDistance(BlockDistance::HALFWAY);
}

Vector2 Attack::calculateGoalTarget() noexcept {
    // Position of the ball from which the goal target is determined
    auto sourcePoint = world->getWorld().value().getBall().value()->getPos();

    // Get the longest line section on the visible part of the goal
    std::vector<LineSegment> openSegments = FieldComputations::getVisiblePartsOfGoal(field, false, sourcePoint, world->getWorld().value().getUs());

    // If there is no empty location to shoot at, just shoot at the center of the goal
    if (openSegments.empty()) return field.getTheirGoalCenter();

    // The longest open segment of the goal will be the best to shoot at
    LineSegment bestSegment = getLongestSegment(openSegments);

    // Make two aim points which are in the corners, since these points are harder for the keeper to intercept
    LineSegment aimPoints = getAimPoints(field, sourcePoint);
    auto leftPoint = aimPoints.start;
    auto rightPoint = aimPoints.end;

    // Check if the left and right points are in the best segment
    double maxY = std::max(bestSegment.start.y, bestSegment.end.y);
    double minY = std::min(bestSegment.start.y, bestSegment.end.y);
    bool leftPointInSegment = leftPoint.y < maxY && leftPoint.y > minY;
    bool rightPointInSegment = rightPoint.y < maxY && rightPoint.y > minY;

    // If we can aim on only one of the points, aim there, otherwise we want to aim for the centre of the largest open segment
    if (leftPointInSegment && rightPointInSegment) {
        // Open goal (mostly), so just shoot in the middle of the largest open segment
        return (bestSegment.start + bestSegment.end) * 0.5;
    } else if (leftPointInSegment) {
        return leftPoint;
    } else if (rightPointInSegment) {
        return rightPoint;
    } else {
        return (bestSegment.start + bestSegment.end) * 0.5;
    }
}

LineSegment Attack::getAimPoints(const world::Field &field, const Vector2 &sourcePoint) {
    LineSegment goalSides = FieldComputations::getGoalSides(field, false);

    // Aim points are located some distance away from the edges of the goal to take into account inaccuracies in the shot
    const double angleMargin = sin(2.0 / 180.0 * M_PI);
    const double constantMargin = 0.05 * field.getGoalWidth();
    Vector2 leftPoint(goalSides.start.x, goalSides.start.y + constantMargin + angleMargin * goalSides.start.dist(sourcePoint));
    Vector2 rightPoint(goalSides.end.x, goalSides.end.y - angleMargin * goalSides.end.dist(sourcePoint) - constantMargin);

    return LineSegment(leftPoint, rightPoint);
}

const LineSegment &Attack::getLongestSegment(const std::vector<LineSegment> &openSegments) {
    unsigned bestIndex = 0;
    for (unsigned i = 1; i < openSegments.size(); i++) {
        auto segment = openSegments[i];
        auto bestSegment = openSegments[bestIndex];
        if (fabs(segment.start.y - segment.end.y) > fabs(bestSegment.start.y - bestSegment.end.y)) {
            bestIndex = i;
        }
    }
    return openSegments[bestIndex];
}

bool Attack::shouldRoleSkipEndTactic() { return false; }

const char *Attack::getName() { return "Attack"; }

}  // namespace rtt::ai::stp::play
