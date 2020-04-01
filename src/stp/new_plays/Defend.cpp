//
// Created by jordi on 27-03-20.
//

#include "stp/new_plays/Defend.h"
#include "stp/new_roles/Defender.h"
#include "stp/new_roles/TestRole.h"

namespace rtt::ai::stp::play {

// TODO: Implement this play, this was just for testing purposes

Defend::Defend() : Play() {
    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
            std::make_unique<role::Defender>(role::Defender("defender_1")), std::make_unique<role::Defender>(role::Defender("defender_2")),
            std::make_unique<TestRole>(TestRole("test_role_2")),      std::make_unique<TestRole>(TestRole("test_role_3")),
            std::make_unique<TestRole>(TestRole("test_role_4")),    std::make_unique<TestRole>(TestRole("test_role_5")),
            std::make_unique<TestRole>(TestRole("test_role_6")),    std::make_unique<TestRole>(TestRole("test_role_7")),
            std::make_unique<TestRole>(TestRole("test_role_8")),    std::make_unique<TestRole>(TestRole("test_role_9")),
            std::make_unique<TestRole>(TestRole("test_role_10"))};
}

uint8_t Defend::score(world_new::World* world) noexcept { return 100; }

Dealer::FlagMap Defend::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"defender_1", {closeToBallFlag}});
    flagMap.insert({"defender_2", {closeToTheirGoalFlag}});
    /*flagMap.insert({"test_role_2", {notImportant}});
    flagMap.insert({"test_role_3", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_4", {closeToBallFlag}});
    flagMap.insert({"test_role_5", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_6", {closeToBallFlag}});
    flagMap.insert({"test_role_7", {closeToTheirGoalFlag}});
    flagMap.insert({"test_role_8", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"test_role_9", {closeToBallFlag}});
    flagMap.insert({"test_role_10", {closeToTheirGoalFlag}});*/

    return flagMap;
}

void Defend::calculateInfoForRoles() noexcept {
    for (auto &role : roles) {
        auto roleName{role->getName()};
        if (stpInfos.find(roleName) != stpInfos.end()) {
            if (stpInfos[roleName].getRobot().value()->getId() == world->getWorld()->getRobotClosestToBall(world_new::us)->getId()
                && world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
                // If current tactic is BlockRobot, force to tactic Intercept
                if (strcmp(role->getCurrentTactic()->getName(), "Block Robot") == 0) role->forceNextTactic();
                stpInfos[roleName].setPositionToMoveTo(world->getWorld()->getBall().value()->getPos());
            } else {
                stpInfos[roleName].setPositionToDefend(field.getOurGoalCenter());
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getThem().at(stpInfos[roleName].getRobot().value()->getId()));
                stpInfos[roleName].setBlockDistance(HALFWAY);
            }
        }
    }
}

bool Defend::isValidPlayToStart(world_new::World* world) noexcept { return true; }

bool Defend::shouldRoleSkipEndTactic() { return false; }

const char *Defend::getName() {
    return "Defend";
}

}  // namespace rtt::ai::stp::play
