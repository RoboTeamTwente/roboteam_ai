//
// Created by jordi on 24-03-20.
//

#include "include/roboteam_ai/stp/new_plays/Attack.h"
#include "include/roboteam_ai/stp/new_roles/Attacker.h"
#include "include/roboteam_ai/stp/new_roles/TestRole.h"

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
    return field.getTheirGoalCenter();
}

bool Attack::isValidPlayToStart(world_new::World* world) noexcept { return true; }

bool Attack::isValidPlayToKeep(world_new::World* world) noexcept { return true; }

bool Attack::shouldRoleSkipEndTactic() { return false; }

} // namespace rtt::ai::stp::play