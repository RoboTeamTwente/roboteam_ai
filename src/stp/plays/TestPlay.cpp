//
// Created by timovdk on 3/10/20.
//

#include "stp/plays/TestPlay.h"

#include "stp/roles/TestRole.h"
#include "stp/roles/Halt.h"

namespace rtt::ai::stp {

TestPlay::TestPlay() : Play() {
    startPlayInvariants.clear();
    // startPlayInvariants.emplace_back();

    keepPlayInvariants.clear();
    // keepPlayInvariants.emplace_back();

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<TestRole>(TestRole("test_role_0")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_1")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_2")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_3")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_4")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_5")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_6")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_7")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_8")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_9")),
        std::make_unique<stp::role::Halt>(stp::role::Halt("halt_role_10"))};
}

uint8_t TestPlay::score(world::World *world) noexcept { return 0; }

Dealer::FlagMap TestPlay::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag notImportantFlag(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"test_role_0", {closeToBallFlag}});
    flagMap.insert({"halt_role_1", {notImportantFlag}});
    flagMap.insert({"halt_role_2", {notImportantFlag}});
    flagMap.insert({"halt_role_3", {notImportantFlag}});
    flagMap.insert({"halt_role_4", {notImportantFlag}});
    flagMap.insert({"halt_role_5", {notImportantFlag}});
    flagMap.insert({"halt_role_6", {notImportantFlag}});
    flagMap.insert({"halt_role_7", {notImportantFlag}});
    flagMap.insert({"halt_role_8", {notImportantFlag}});
    flagMap.insert({"halt_role_9", {notImportantFlag}});
    flagMap.insert({"halt_role_10", {notImportantFlag}});

    return flagMap;
}

void TestPlay::calculateInfoForRoles() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["test_role_0"].setPositionToMoveTo(Vector2{-length,-width/2});
}

bool TestPlay::shouldRoleSkipEndTactic() { return false; }

const char *TestPlay::getName() { return "Test Play"; }

}  // namespace rtt::ai::stp
