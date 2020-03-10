//
// Created by timovdk on 3/10/20.
//

#include <stp/new_plays/TestPlay.h>
#include <stp/new_roles/TestRole.h>

namespace rtt::ai::stp {

TestPlay::TestPlay() {
    roles = std::array<std::unique_ptr<Role>, ROBOT_COUNT>{
        std::make_unique<Role>(TestRole("test_role_0")), std::make_unique<Role>(TestRole("test_role_1")), std::make_unique<Role>(TestRole("test_role_2")),
        std::make_unique<Role>(TestRole("test_role_3")), std::make_unique<Role>(TestRole("test_role_4")), std::make_unique<Role>(TestRole("test_role_5")),
        std::make_unique<Role>(TestRole("test_role_6")), std::make_unique<Role>(TestRole("test_role_7"))/*, std::make_unique<Role>(TestRole("test_role_8")),
        std::make_unique<Role>(TestRole("test_role_9")), std::make_unique<Role>(TestRole("test_role_10"))*/};
}

bool TestPlay::isValidPlay(world_new::World* world) noexcept { return true; }

uint8_t TestPlay::score(world_new::World* world) noexcept { return 10; }

void TestPlay::assignRoles() noexcept {
    Dealer dealer{world->getWorld().value(), &field};

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
    /*    flagMap.insert({"test_role_8", {closeToTheirGoalFlag, closeToBallFlag}});
        flagMap.insert({"test_role_9", {closeToBallFlag}});
        flagMap.insert({"test_role_10", {closeToTheirGoalFlag}});*/

    auto distribution = dealer.distribute(world->getWorld()->getUs(), flagMap);

    for (int i = 0; i < roles.size(); i++) {
        if (distribution.find(roles[i]->getName()) != distribution.end()) {
            tacticInfos[i].setRobot(distribution.find(roles[i]->getName())->second);
            tacticInfos[i].setField(*world->getField());

            // TODO calculate additional info
            // setTacticInfo(info, role->getName());
        }
    }
}
}  // namespace rtt::ai::stp
