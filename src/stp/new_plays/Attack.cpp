//
// Created by jordi on 24-03-20.
//

#include "stp/new_plays/Attack.h"
#include "stp/new_roles/Attacker.h"
#include "stp/new_roles/TestRole.h"

namespace rtt::ai::stp::play {

Attack::Attack() {
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::Attacker>(role::Attacker("attacker")),   std::make_unique<TestRole>(TestRole("test_role_1")),
            std::make_unique<TestRole>(TestRole("test_role_2")),            std::make_unique<TestRole>(TestRole("test_role_3")),
            std::make_unique<TestRole>(TestRole("test_role_4")),            std::make_unique<TestRole>(TestRole("test_role_5")),
            std::make_unique<TestRole>(TestRole("test_role_6")),            std::make_unique<TestRole>(TestRole("test_role_7")),
            std::make_unique<TestRole>(TestRole("test_role_8")),            std::make_unique<TestRole>(TestRole("test_role_9")),
            std::make_unique<TestRole>(TestRole("test_role_10"))};
}

uint8_t Attack::score(world_new::World* world) noexcept { return 100; }

void Attack::assignRoles() noexcept {
    Dealer dealer{world->getWorld().value(), &field};

    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"attacker", {closeToBallFlag}});
    /*flagMap.insert({"test_role_1", {notImportant}});
    flagMap.insert({"test_role_2", {notImportant}});
    flagMap.insert({"test_role_3", {notImportant}});
    flagMap.insert({"test_role_4", {notImportant}});
    flagMap.insert({"test_role_5", {notImportant}});
    flagMap.insert({"test_role_6", {notImportant}});
    flagMap.insert({"test_role_7", {notImportant}});
    flagMap.insert({"test_role_8", {notImportant}});
    flagMap.insert({"test_role_9", {notImportant}});
    flagMap.insert({"test_role_10", {notImportant}});*/

    auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap);

    stpInfos = std::unordered_map<std::string, StpInfo>{};
    for (auto& role : roles) {
        auto roleName{role->getName()};
        if (distribution.find(roleName) != distribution.end()) {
            auto robot = distribution.find(role->getName())->second;

            stpInfos.emplace(roleName, StpInfo{});
            stpInfos[roleName].setRobot(robot);
        }
    }
}

void Attack::calculateInfoForRoles() noexcept {
    auto goalTarget = calculateGoalTarget();

    // Calculate attacker info
    if (stpInfos.find("attacker") != stpInfos.end()) {
        stpInfos["attacker"].setPositionToShootAt(goalTarget);
        stpInfos["attacker"].setKickChipType(MAX_SPEED);
    }
}

Vector2 Attack::calculateGoalTarget() noexcept {
    // Position of the ball from which the goal target is determined
    auto sourcePoint = world->getWorld().value().getBall().value()->getPos();

    // Get the longest line section on the visible part of the goal
    std::vector<Line> openSegments = FieldComputations::getVisiblePartsOfGoal(field, false, sourcePoint, world->getWorld().value());

    // If there is no empty location to shoot at, just shoot at the center of the goal
    if (openSegments.empty()) return field.getTheirGoalCenter();

    // The longest open segment of the goal will be the best to shoot at
    auto bestSegment = getLongestSegment(openSegments);

    // Make two aim points which are in the corners, since these points are harder for the keeper to intercept
    Line aimPoints = getAimPoints(field, sourcePoint);
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

Line Attack::getAimPoints(const Field &field, const Vector2 &sourcePoint) {
    Line goalSides = FieldComputations::getGoalSides(field, false);

    // Aim points are located some distance away from the edges of the goal to take into account inaccuracies in the shot
    const double angleMargin = sin(2.0 / 180.0 * M_PI);
    const double constantMargin = 0.05 * field.getGoalWidth();
    Vector2 leftPoint(goalSides.start.x, goalSides.start.y + constantMargin + angleMargin * goalSides.start.dist(sourcePoint));
    Vector2 rightPoint(goalSides.end.x, goalSides.end.y - angleMargin * goalSides.end.dist(fromPoint) - constantMargin);

    return Line(leftPoint, rightPoint);
}

const Line &Attack::getLongestSegment(const std::vector<Line> &openSegments) {
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

bool Attack::isValidPlayToStart(world_new::World* world) noexcept { return true; }

bool Attack::isValidPlayToKeep(world_new::World* world) noexcept { return true; }

bool Attack::shouldRoleSkipEndTactic() { return false; }

} // namespace rtt::ai::stp::play
