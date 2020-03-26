//
// Created by timovdk on 3/10/20.
//

#include <stp/new_plays/TestPlay.h>
#include <stp/new_roles/TestRole.h>

namespace rtt::ai::stp {

TestPlay::TestPlay() {
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<TestRole>(TestRole("test_role_0")), std::make_unique<TestRole>(TestRole("test_role_1")), std::make_unique<TestRole>(TestRole("test_role_2")),
        std::make_unique<TestRole>(TestRole("test_role_3")), std::make_unique<TestRole>(TestRole("test_role_4")), std::make_unique<TestRole>(TestRole("test_role_5")),
        std::make_unique<TestRole>(TestRole("test_role_6")), std::make_unique<TestRole>(TestRole("test_role_7")), std::make_unique<TestRole>(TestRole("test_role_8")),
        std::make_unique<TestRole>(TestRole("test_role_9")), std::make_unique<TestRole>(TestRole("test_role_10"))};
}

uint8_t TestPlay::score(world_new::World *world) noexcept { return 10; }

Dealer::FlagMap TestPlay::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"test_role_0", {closeToBallFlag}});
    flagMap.insert({"test_role_1", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_2", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_3", {closeToBallFlag}});
    flagMap.insert({"test_role_4", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_5", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_6", {closeToBallFlag}});
    flagMap.insert({"test_role_7", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_8", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_9", {closeToBallFlag}});
    flagMap.insert({"test_role_10", {closeToTheirGoalFlag}});

    return flagMap;
}

void TestPlay::calculateInfoForRoles() noexcept {
    for (auto &role : roles) {
        auto roleName{role->getName()};
        if (stpInfos.find(roleName) != stpInfos.end()) {
            // TODO when deciding the intercept position, there should be some compensation for movement of the ball and reaction times, up to control I guess
            stpInfos[roleName].setPositionToMoveTo(world->getWorld()->getBall()->get()->getPos() + world->getWorld()->getBall()->get()->getFilteredVelocity() * 0.5);
        }
    }
}

bool TestPlay::isValidPlayToStart(world_new::World *world) noexcept { return false; }

bool TestPlay::isValidPlayToKeep(world_new::World *world) noexcept { return false; }

bool TestPlay::shouldRoleSkipEndTactic() { return false; }

}  // namespace rtt::ai::stp
