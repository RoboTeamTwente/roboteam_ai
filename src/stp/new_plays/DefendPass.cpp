//
// Created by jordi on 14-05-20.
//

#include "stp/new_plays/DefendPass.h"

#include "stp/invariants/BallOnOurSideInvariant.h"

#include "stp/invariants/BallCloseToThemInvariant.h"
#include "stp/invariants/BallShotOrCloseToThemInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "stp/new_roles/Defender.h"
#include "stp/new_roles/Formation.h"
#include "stp/new_roles/Harasser.h"
#include "stp/new_roles/Keeper.h"

namespace rtt::ai::stp::play {

DefendPass::DefendPass() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToThemInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallOnOurSideInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallShotOrCloseToThemInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),          std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")),  std::make_unique<role::Defender>(role::Defender("blocker_1")),
        std::make_unique<role::Defender>(role::Defender("blocker_2")),   std::make_unique<role::Defender>(role::Defender("blocker_3")),
        std::make_unique<role::Defender>(role::Defender("blocker_4")),   std::make_unique<role::Defender>(role::Defender("blocker_5")),
        std::make_unique<role::Harasser>(role::Harasser("harasser")),    std::make_unique<role::Formation>(role::Formation("offender_1")),
        std::make_unique<role::Formation>(role::Formation("offender_2"))};
}

uint8_t DefendPass::score(world_new::World *world) noexcept { return 100; }

Dealer::FlagMap DefendPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::LOW_PRIORITY);
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    flagMap.insert({"blocker_1", {closeToOurGoalFlag}});
    flagMap.insert({"blocker_2", {closeToOurGoalFlag}});
    flagMap.insert({"blocker_3", {not_important}});
    flagMap.insert({"blocker_4", {not_important}});
    flagMap.insert({"blocker_5", {not_important}});
    flagMap.insert({"harasser", {closeToBallFlag}});
    flagMap.insert({"offender_1", {closeToTheirGoalFlag}});
    flagMap.insert({"offender_2", {closeToTheirGoalFlag}});

    return flagMap;
}

void DefendPass::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForBlockers();
    calculateInfoForKeeper();
    calculateInfoForHarassers();
    calculateInfoForOffenders();
}

void DefendPass::calculateInfoForDefenders() noexcept {
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world_new::them);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_1"].setBlockDistance(HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_2"].setBlockDistance(HALFWAY);

    // When the ball moves, one defender tries to intercept the ball
    for (auto &role : roles) {
        auto roleName = role->getName();
        if (roleName.find("defender") != std::string::npos) {
            // TODO: Improve choice of intercept robot based on trajectory and intercept position
            if (world->getWorld()->getRobotClosestToBall(world_new::us) && stpInfos[roleName].getRobot() &&
                stpInfos[roleName].getRobot().value()->getId() == world->getWorld()->getRobotClosestToBall(world_new::us).value()->getId() &&
                world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL) {
                // If current tactic is BlockRobot, force to tactic Intercept
                if (strcmp(role->getCurrentTactic()->getName(), "Block Robot") == 0) role->forceNextTactic();
                // TODO: Improve intercept position
                stpInfos[roleName].setPositionToMoveTo(world->getWorld()->getBall().value()->getPos());
            }
        }
    }
}

void DefendPass::calculateInfoForBlockers() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyPasser = world->getWorld()->getRobotClosestToBall(world_new::them);

    if(enemyRobots.empty()) {
      RTT_ERROR("There are no enemy robots, which are necessary for this play!")
      return;
    }

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(), [&](const auto enemyRobot) -> bool { return enemyPasser && enemyRobot->getId() == enemyPasser.value()->getId(); }));

    for (auto &stpInfo : stpInfos) {
        if (stpInfo.first.find("blocker") != std::string::npos) {
            auto roleName = stpInfo.first;

            if (!enemyRobots.empty()) {
                // If there are enemy robots available, block the closest robot to our goal
                auto enemyToBlock = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

                enemyRobots.erase(
                    std::remove_if(enemyRobots.begin(), enemyRobots.end(), [&](const auto enemyRobot) -> bool { return enemyToBlock && enemyRobot->getId() == enemyToBlock.value()->getId(); }));

                if(enemyToBlock) {
                    stpInfos[roleName].setPositionToDefend(enemyToBlock.value()->getPos());
                }
                else {
                    stpInfos[roleName].setPositionToDefend(std::nullopt);
                }

                stpInfos[roleName].setEnemyRobot(enemyPasser);
                stpInfos[roleName].setBlockDistance(FAR);
            } else {
                // TODO: Improve default behaviour when there are no enemy robots to block
                stpInfos[roleName].setPositionToDefend(field.getOurGoalCenter());
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world_new::them));
                stpInfos[roleName].setBlockDistance(HALFWAY);
            }
        }
    }
}

void DefendPass::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void DefendPass::calculateInfoForHarassers() noexcept { stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them)); }

void DefendPass::calculateInfoForOffenders() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["offender_1"].setPositionToMoveTo(Vector2(length / 4, width / 6));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(length / 4, -width / 6));
}

bool DefendPass::shouldRoleSkipEndTactic() { return false; }

const char *DefendPass::getName() { return "Defend Pass"; }

}  // namespace rtt::ai::stp::play