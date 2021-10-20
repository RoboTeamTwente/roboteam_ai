//
// Created by jordi on 14-05-20.
//

#include "stp/plays/defensive/DefendPass.h"

#include "stp/roles/passive/Defender.h"
#include "stp/roles/passive/Formation.h"
#include "stp/roles/passive/Harasser.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

DefendPass::DefendPass() : Play() {
    startPlayEvaluation.clear();
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    startPlayEvaluation.emplace_back(eval::BallCloseToThem);
    startPlayEvaluation.emplace_back(eval::BallOnOurSide);

    keepPlayEvaluation.clear();
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
    keepPlayEvaluation.emplace_back(eval::BallShotOrCloseToThem);

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
        std::make_unique<role::Keeper>(role::Keeper("keeper")),
        std::make_unique<role::Defender>(role::Defender("defender_1")),
        std::make_unique<role::Defender>(role::Defender("defender_2")),
        std::make_unique<role::Defender>(role::Defender("blocker_1")),
        std::make_unique<role::Defender>(role::Defender("blocker_2")),
        std::make_unique<role::Defender>(role::Defender("blocker_3")),
        std::make_unique<role::Defender>(role::Defender("blocker_4")),
        std::make_unique<role::Defender>(role::Defender("blocker_5")),
        std::make_unique<role::Harasser>(role::Harasser("harasser")),
        std::make_unique<role::Formation>(role::Formation("offender_1")),
        std::make_unique<role::Formation>(role::Formation("offender_2"))};
}

uint8_t DefendPass::score(PlayEvaluator &playEvaluator) noexcept { return 100; }

Dealer::FlagMap DefendPass::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::LOW_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER,{}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::REQUIRED,{closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::REQUIRED,{closeToOurGoalFlag}}});
    flagMap.insert({"blocker_1", {DealerFlagPriority::HIGH_PRIORITY,{closeToOurGoalFlag}}});
    flagMap.insert({"blocker_2", {DealerFlagPriority::HIGH_PRIORITY,{closeToOurGoalFlag}}});
    flagMap.insert({"blocker_3", {DealerFlagPriority::LOW_PRIORITY,{notImportant}}});
    flagMap.insert({"blocker_4", {DealerFlagPriority::LOW_PRIORITY,{notImportant}}});
    flagMap.insert({"blocker_5", {DealerFlagPriority::LOW_PRIORITY,{notImportant}}});
    flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
    flagMap.insert({"offender_1", {DealerFlagPriority::MEDIUM_PRIORITY,{closeToTheirGoalFlag}}});
    flagMap.insert({"offender_2", {DealerFlagPriority::MEDIUM_PRIORITY,{closeToTheirGoalFlag}}});

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
    auto enemyClosestToBall = world->getWorld()->getRobotClosestToBall(world::them);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::CLOSE);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setEnemyRobot(enemyClosestToBall);
    stpInfos["defender_2"].setBlockDistance(BlockDistance::CLOSE);

    // When the ball moves, one defender tries to intercept the ball
    auto closestBotUs = world->getWorld()->getRobotClosestToBall(world::us);
    auto closestBotThem = world->getWorld()->getRobotClosestToBall(world::them);
    for (auto &role : roles) {
        auto roleName = role->getName();
        if (closestBotUs && closestBotThem && roleName.find("defender") != std::string::npos) {
            // TODO: Improve choice of intercept robot based on trajectory and intercept position
            if (stpInfos[roleName].getRobot() && stpInfos[roleName].getRobot().value()->getId() == closestBotUs.value()->getId() &&
                world->getWorld()->getBall().value()->getVelocity().length() > control_constants::BALL_STILL_VEL &&
                closestBotThem->get()->getDistanceToBall() > control_constants::BALL_IS_CLOSE) {
                // If current tactic is BlockRobot, force to tactic Intercept
                if (strcmp(role->getCurrentTactic()->getName(), "Block Robot") == 0) role->forceNextTactic();
                // TODO: Improve intercept position
                stpInfos[roleName].setPositionToMoveTo(world->getWorld()->getBall().value()->getPos());
            }
        }
    }
}

// TODO-Max move to tactics
void DefendPass::calculateInfoForBlockers() noexcept {
    auto enemyRobots = world->getWorld()->getThem();
    auto enemyPasser = world->getWorld()->getRobotClosestToBall(world::them);

    if (enemyRobots.empty()) {
        RTT_ERROR("There are no enemy robots, which are necessary for this play!")
        return;
    }

    enemyRobots.erase(
        std::remove_if(enemyRobots.begin(), enemyRobots.end(), [&](const auto enemyRobot) -> bool { return enemyPasser && enemyRobot->getId() == enemyPasser.value()->getId(); }));

    for (auto &stpInfo : stpInfos) {
        if (stpInfo.first.find("blocker") != std::string::npos) {
            auto roleName = stpInfo.first;

            if (!enemyRobots.empty()) {
                // If there are enemy robots available, block the closest robot to our goal
                auto enemyToBlock = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

                enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                                 [&](const auto enemyRobot) -> bool { return enemyToBlock && enemyRobot->getId() == enemyToBlock.value()->getId(); }));

                if (enemyToBlock) {
                    stpInfos[roleName].setPositionToDefend(enemyToBlock.value()->getPos());
                } else {
                    stpInfos[roleName].setPositionToDefend(std::nullopt);
                }

                stpInfos[roleName].setEnemyRobot(enemyPasser);
                stpInfos[roleName].setBlockDistance(BlockDistance::FAR);
            } else {
                // TODO: Improve default behaviour when there are no enemy robots to block
                stpInfos[roleName].setPositionToDefend(field.getOurGoalCenter());
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
                stpInfos[roleName].setBlockDistance(BlockDistance::CLOSE);
            }
        }
    }
}

void DefendPass::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void DefendPass::calculateInfoForHarassers() noexcept { stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them)); }

void DefendPass::calculateInfoForOffenders() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["offender_1"].setPositionToMoveTo(Vector2(length / 4, width / 6));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(length / 4, -width / 6));
}

const char *DefendPass::getName() { return "Defend Pass"; }

}  // namespace rtt::ai::stp::play