//
// Created by jordi on 27-03-20.
//

#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/invariants/BallOnOurSideInvariant.h"
#include "stp/invariants/BallCloseToThemInvariant.h"
#include "stp/new_plays/Defend.h"
#include "stp/new_roles/Defender.h"
#include "stp/new_roles/Keeper.h"
#include "stp/new_roles/Formation.h"

namespace rtt::ai::stp::play {

Defend::Defend() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallOnOurSideInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToThemInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallOnOurSideInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),
        std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")),
        std::make_unique<role::Defender>(role::Defender("defender_3")),
        std::make_unique<role::Defender>(role::Defender("defender_4")),
        std::make_unique<role::Formation>(role::Formation("midfielder_1")),
        std::make_unique<role::Formation>(role::Formation("midfielder_2")),
        std::make_unique<role::Formation>(role::Formation("midfielder_3")),
        std::make_unique<role::Formation>(role::Formation("midfielder_4")),
        std::make_unique<role::Formation>(role::Formation("offender_1")),
        std::make_unique<role::Formation>(role::Formation("offender_2"))};
}

uint8_t Defend::score(world_new::World* world) noexcept { return 50; }

Dealer::FlagMap Defend::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::REQUIRED);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    flagMap.insert({"defender_3", {closeToOurGoalFlag}});
    flagMap.insert({"defender_4", {closeToOurGoalFlag}});
    flagMap.insert({"midfielder_1", {}});
    flagMap.insert({"midfielder_2", {}});
    flagMap.insert({"midfielder_3", {}});
    flagMap.insert({"midfielder_4", {}});
    flagMap.insert({"offender_1", {closeToTheirGoalFlag}});
    flagMap.insert({"offender_2", {closeToTheirGoalFlag}});

    return flagMap;
}

void Defend::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForKeeper();
    calculateInfoForMidfielders();
    calculateInfoForOffenders();
}

void Defend::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyAttacker = world->getWorld()->getRobotClosestToBall(world_new::them);

    for (auto enemyRobot = enemyRobots.begin(); enemyRobot < enemyRobots.end(); enemyRobot++) {
        if (enemyRobot->get()->getId() == enemyAttacker->getId()) {
            enemyRobots.erase(enemyRobot);
        }
    }

    auto enemyClosestToGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_1"].setBlockDistance(HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_2"].setEnemyRobot(enemyClosestToGoal);
    stpInfos["defender_2"].setBlockDistance(HALFWAY);

    stpInfos["defender_3"].setPositionToDefend(enemyClosestToGoal->getPos());
    stpInfos["defender_3"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_3"].setBlockDistance(HALFWAY);

    stpInfos["defender_4"].setPositionToDefend(field.getOurGoalCenter() + Vector2(4*control_constants::ROBOT_RADIUS, 0));
    stpInfos["defender_4"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_4"].setBlockDistance(HALFWAY);

    // When the ball moves, one defender tries to intercept the ball
    for (auto &role : roles) {
        auto roleName = role->getName();
        if (roleName.find("defender") != std::string::npos) {
            // TODO: Improve choice of intercept robot based on trajectory and intercept position
            if (stpInfos[roleName].getRobot().has_value()
                && stpInfos[roleName].getRobot().value()->getId() == world->getWorld()->getRobotClosestToBall(world_new::us)->getId()
                && world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
                // If current tactic is BlockRobot, force to tactic Intercept
                if (strcmp(role->getCurrentTactic()->getName(), "Block Robot") == 0) role->forceNextTactic();
                // TODO: Improve intercept position
                stpInfos[roleName].setPositionToMoveTo(world->getWorld()->getBall().value()->getPos());
            }
        }
    }
}

void Defend::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void Defend::calculateInfoForMidfielders() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["midfielder_1"].setPositionToMoveTo(Vector2(0.0, width/4));
    stpInfos["midfielder_2"].setPositionToMoveTo(Vector2(0.0, -width/4));
    stpInfos["midfielder_3"].setPositionToMoveTo(Vector2(-length/8, 0.0));
    stpInfos["midfielder_4"].setPositionToMoveTo(Vector2(length/8, 0.0));
}

void Defend::calculateInfoForOffenders() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["offender_1"].setPositionToMoveTo(Vector2(length/4, width/6));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(length/4, -width/6));
}

bool Defend::shouldRoleSkipEndTactic() { return false; }

const char *Defend::getName() {
    return "Defend";
}

}  // namespace rtt::ai::stp::play
