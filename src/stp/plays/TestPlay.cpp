//
// Created by timovdk on 3/10/20.
//

/// This play is for testing purposes, to use it, be sure to uncomment in in STPManager.cpp's start function!

#include "stp/plays/TestPlay.h"

#include "stp/roles/TestRole.h"
#include "stp/roles/active/BallInterceptor.h"

namespace rtt::ai::stp {

TestPlay::TestPlay() : Play() {
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::BallInterceptor>("ballInterceptor"), std::make_unique<TestRole>("role_1"), std::make_unique<TestRole>("role_2"), std::make_unique<TestRole>("role_3"),
        std::make_unique<TestRole>("role_4"), std::make_unique<TestRole>("role_5"), std::make_unique<TestRole>("role_6"), std::make_unique<TestRole>("role_7"),
        std::make_unique<TestRole>("role_8"), std::make_unique<TestRole>("role_9"), std::make_unique<TestRole>("role_10")};
}

uint8_t TestPlay::score(const rtt::world::Field& field) noexcept { return 0; }

Dealer::FlagMap TestPlay::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"ballInterceptor", {DealerFlagPriority::HIGH_PRIORITY, {}}});
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

void TestPlay::calculateInfoForRoles() noexcept {
    stpInfos["ballInterceptor"].setPositionToMoveTo(Vector2());
}

const char* TestPlay::getName() { return "Test Play"; }

}  // namespace rtt::ai::stp
