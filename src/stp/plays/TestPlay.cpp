//
// Created by timovdk on 3/10/20.
//

#include "stp/plays/TestPlay.h"

#include "stp/roles/TestRole.h"

namespace rtt::ai::stp {

TestPlay::TestPlay() : Play() {
    startPlayEvaluation.clear();
    // keepPlayEvaluation.emplace_back();

    keepPlayEvaluation.clear();
    // keepPlayEvaluation.emplace_back();

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<TestRole>(TestRole("test_role_0")),
        std::make_unique<TestRole>(TestRole("test_role_1")),
        std::make_unique<TestRole>(TestRole("test_role_2")),
        std::make_unique<TestRole>(TestRole("test_role_3")),
        std::make_unique<TestRole>(TestRole("test_role_4")),
        std::make_unique<TestRole>(TestRole("test_role_5")),
        std::make_unique<TestRole>(TestRole("test_role_6")),
        std::make_unique<TestRole>(TestRole("test_role_7")),
        std::make_unique<TestRole>(TestRole("test_role_8")),
        std::make_unique<TestRole>(TestRole("test_role_9")),
        std::make_unique<TestRole>(TestRole("test_role_10"))};
}

uint8_t TestPlay::score(PlayEvaluator& playEvaluator) noexcept { return 0; }

Dealer::FlagMap TestPlay::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"test_role_0", {DealerFlagPriority::REQUIRED, {closeToBallFlag,}}});
    flagMap.insert({"test_role_1", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag}}});
    flagMap.insert({"test_role_2", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag, closeToBallFlag}}});
    flagMap.insert({"test_role_3", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"test_role_4", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag}}});
    flagMap.insert({"test_role_5", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag, closeToBallFlag}}});
    flagMap.insert({"test_role_6", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"test_role_7", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag}}});
    flagMap.insert({"test_role_8", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag, closeToBallFlag}}});
    flagMap.insert({"test_role_9", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"test_role_10", {DealerFlagPriority::REQUIRED, {closeToTheirGoalFlag}}});

    return flagMap;
}

void TestPlay::calculateInfoForRoles() noexcept {}
void TestPlay::calculateInfoForScoredRoles(world::World *world) noexcept {}

const char *TestPlay::getName() { return "Test Play"; }

}  // namespace rtt::ai::stp
