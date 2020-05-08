//
// Created by jessevw on 17.03.20.
//

#include "stp/new_plays/Pass.h"

#include "stp/invariants/BallCloseToUsInvariant.h"
#include "stp/invariants/WeHaveBallInvariant.h"
#include "stp/new_roles/TestRole.h"

#include "stp/invariants/BallMovesSlowInvariant.h"
#include "stp/new_roles/PassReceiver.h"
#include "stp/new_roles/Passer.h"

namespace rtt::ai::stp::play {

Pass::Pass() : Play() {
    // TODO: decide start invariants
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());

    // TODO: decide keep invariants
    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallMovesSlowInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Passer>(role::Passer("passer")), std::make_unique<role::PassReceiver>(role::PassReceiver("pass_receiver")),
        std::make_unique<TestRole>(TestRole("defender1")),      std::make_unique<TestRole>(TestRole("test_role_3")),
        std::make_unique<TestRole>(TestRole("test_role_4")),    std::make_unique<TestRole>(TestRole("test_role_5")),
        std::make_unique<TestRole>(TestRole("test_role_6")),    std::make_unique<TestRole>(TestRole("test_role_7")),
        std::make_unique<TestRole>(TestRole("test_role_8")),    std::make_unique<TestRole>(TestRole("test_role_9")),
        std::make_unique<TestRole>(TestRole("test_role_10"))};
}

uint8_t Pass::score(world_new::World* world) noexcept { return -10; }

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
    if (stpInfos.find("pass_receiver") != stpInfos.end()) stpInfos["pass_receiver"].setPositionToMoveTo(passingPosition);
    // Calculate Passer info
    if (stpInfos.find("passer") != stpInfos.end()) {
        stpInfos["passer"].setPositionToShootAt(passingPosition);
        stpInfos["passer"].setKickChipType(PASS);
    }

    for (int defenderIndex = 0; defenderIndex < numberOfDefenders; defenderIndex++) {
        std::string defenderName = "defender" + std::to_string(defenderIndex + 1);

        if (stpInfos.find(defenderName) != stpInfos.end()) {
            stpInfos[defenderName].setPositionToMoveTo(defensivePositions[defenderIndex]);
        }
    }
    if (stpInfos.find("test_role_3") != stpInfos.end()) stpInfos["test_role_3"].setPositionToMoveTo(Vector2{-3, -3});
    if (stpInfos.find("test_role_4") != stpInfos.end()) stpInfos["test_role_4"].setPositionToMoveTo(Vector2{-2, 3});
    if (stpInfos.find("test_role_5") != stpInfos.end()) stpInfos["test_role_5"].setPositionToMoveTo(Vector2{-2, 0});
    if (stpInfos.find("test_role_6") != stpInfos.end()) stpInfos["test_role_6"].setPositionToMoveTo(Vector2{-2, -3});
    if (stpInfos.find("test_role_7") != stpInfos.end()) stpInfos["test_role_7"].setPositionToMoveTo(Vector2{-1, 4});
    if (stpInfos.find("test_role_8") != stpInfos.end()) stpInfos["test_role_8"].setPositionToMoveTo(Vector2{-1, 1.5});
    if (stpInfos.find("test_role_9") != stpInfos.end()) stpInfos["test_role_9"].setPositionToMoveTo(Vector2{-1, -1.5});
    if (stpInfos.find("test_role_10") != stpInfos.end()) stpInfos["test_role_10"].setPositionToMoveTo(Vector2{-1, -4});
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

bool Pass::shouldRoleSkipEndTactic() { return false; }

const char* Pass::getName() { return "Pass"; }

}  // namespace rtt::ai::stp::play
