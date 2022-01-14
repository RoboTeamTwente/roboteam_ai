//
// Created by timovdk on 3/10/20.
//

#include "stp/plays/TestPlay.h"

#include "stp/computations/GoalComputations.h"
#include "stp/roles/TestRole.h"
#include "stp/roles/active/Attacker.h"
#include "stp/roles/passive/Halt.h"

namespace rtt::ai::stp {

TestPlay::TestPlay() : Play() {
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<TestRole>("role_0"),   std::make_unique<role::Halt>("role_1"), std::make_unique<role::Halt>("role_2"), std::make_unique<role::Halt>("role_3"),
        std::make_unique<role::Halt>("role_4"), std::make_unique<role::Halt>("role_5"), std::make_unique<role::Halt>("role_6"), std::make_unique<role::Halt>("role_7"),
        std::make_unique<role::Halt>("role_8"), std::make_unique<role::Halt>("role_9"), std::make_unique<role::Halt>("role_10")};
}

uint8_t TestPlay::score(PlayEvaluator &playEvaluator) noexcept { return 0; }

Dealer::FlagMap TestPlay::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"role_0", {DealerFlagPriority::REQUIRED, {}}});
    flagMap.insert({"role_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_4", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_5", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_6", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_7", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_8", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_9", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_10", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void TestPlay::calculateInfoForRoles() noexcept {}

const char *TestPlay::getName() { return "Test Play"; }

}  // namespace rtt::ai::stp
