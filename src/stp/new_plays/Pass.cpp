//
// Created by jessevw on 17.03.20.
//

#include "stp/new_plays/Pass.h"

#include <stp/new_roles/TestRole.h>

#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"

namespace rtt::ai::stp::play {

Pass::Pass() {
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Passer>(role::Passer("passer")), std::make_unique<role::PassReceiver>(role::PassReceiver("pass_receiver")),
        std::make_unique<TestRole>(TestRole("defender1")),      std::make_unique<TestRole>(TestRole("test_role_3")),
        std::make_unique<TestRole>(TestRole("test_role_4")),    std::make_unique<TestRole>(TestRole("test_role_5")),
        std::make_unique<TestRole>(TestRole("test_role_6")),    std::make_unique<TestRole>(TestRole("test_role_7")),
        std::make_unique<TestRole>(TestRole("test_role_8")),    std::make_unique<TestRole>(TestRole("test_role_9")),
        std::make_unique<TestRole>(TestRole("test_role_10"))};
}

uint8_t Pass::score(world_new::World* world) noexcept { return 13; }

Dealer::FlagMap Pass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"passer", {closeToBallFlag}});
    flagMap.insert({"pass_receiver", {closeToTheirGoalFlag}});
    flagMap.insert({"defender1", {notImportant}});
    flagMap.insert({"test_role_3", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_4", {closeToBallFlag}});
    flagMap.insert({"test_role_5", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_6", {closeToBallFlag}});
    flagMap.insert({"test_role_7", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_8", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_9", {closeToBallFlag}});
    flagMap.insert({"test_role_10", {closeToTheirGoalFlag}});

    return flagMap;
}

void Pass::calculateInfoForRoles() noexcept {
    // Calculate most important positions to defend
    // You know you have n defenders, because the play assigned it that way
    auto enemyRobots = world->getWorld()->getThem();
    const int numberOfDefenders = 2;
    auto defensivePositions = calculateDefensivePositions(numberOfDefenders, world, enemyRobots);

    // TODO: is there really no better way to set data per role?
    // Use this new information to assign the roles using the dealer.
    // TODO: compute the passing position
    const Vector2 passingPosition = Vector2(-2, -2);

    // Calculate receiver info
    if (stpInfos.find("pass_receiver") != stpInfos.end())
        stpInfos["pass_receiver"].setPositionToMoveTo(passingPosition);
    // Calculate Passer info
    if (stpInfos.find("passer") != stpInfos.end()) stpInfos["passer"].setPositionToShootAt(passingPosition);

    for (int defenderIndex = 0; defenderIndex < numberOfDefenders; defenderIndex++) {
        std::string defenderName = "defender" + std::to_string(defenderIndex + 1);

        if (stpInfos.find(defenderName) != stpInfos.end()) {
            stpInfos[defenderName].setPositionToMoveTo(defensivePositions[defenderIndex]);
        }
    }
}

std::vector<Vector2> Pass::calculateDefensivePositions(int numberOfDefenders, world_new::World* world, std::vector<world_new::view::RobotView> enemyRobots) {
    std::vector<Vector2> positions = {};
    // 3 robots will defend goal
    for (int i = 0; i < numberOfDefenders; i++) {
        if (i < 3) {
            positions.push_back(world->getField()->getOurGoalCenter());
        } else {
            positions.push_back(enemyRobots[i].get()->getPos());
        }
    }

    return positions;
}

bool Pass::isValidPlayToStart(world_new::World* world) noexcept { return true; }

bool Pass::isValidPlayToKeep(world_new::World* world) noexcept { return true; }

bool Pass::shouldRoleSkipEndTactic() { return false; }

}  // namespace rtt::ai::stp::play
