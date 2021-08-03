//
// Created by jordi on 07-05-20.
//

#include "stp/plays/referee_specific/FreeKickThem.h"
#include "stp/roles/passive/Defender.h"
#include "stp/computations/PositionComputations.h"
#include "stp/roles/Keeper.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {
const char* FreeKickThem::getName() { return "Free Kick Them"; }

FreeKickThem::FreeKickThem() : Play() {
    /// Evaluations that have to be true to be considered when changing plays.
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::FreeKickThemGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::FreeKickThemGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::Keeper>("keeper"),
        std::make_unique<role::Defender>("defender_0"),
        std::make_unique<role::Defender>("defender_1"),
        std::make_unique<role::Defender>("defender_2"),
        std::make_unique<role::Defender>("blocker_0"),
        std::make_unique<role::Defender>("blocker_1"),
        std::make_unique<role::Defender>("blocker_2"),
        std::make_unique<role::Defender>("blocker_3"),
        std::make_unique<role::Defender>("blocker_4"),
        std::make_unique<role::Defender>("blocker_5"),
        std::make_unique<role::Formation>("offender")
    };

    initRoles();  // DONT TOUCH.
}

Dealer::FlagMap FreeKickThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;  // DONT TOUCH.

    /// Flags that have a factor and a weight linked to it, can be given to a role
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);

    /// Creation flagMap. Linking roles to role-priority and the above created flags, can also force ID {roleName, {priority, flags, forceID}}
    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"defender_0", {DealerFlagPriority::LOW_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_1", {DealerFlagPriority::LOW_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"defender_2", {DealerFlagPriority::LOW_PRIORITY, {closeToOurGoalFlag}}});
    flagMap.insert({"blocker_0", {DealerFlagPriority::LOW_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"blocker_1", {DealerFlagPriority::LOW_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"blocker_2", {DealerFlagPriority::LOW_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"blocker_3", {DealerFlagPriority::LOW_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"blocker_4", {DealerFlagPriority::LOW_PRIORITY, {closeToBallFlag}}});
    flagMap.insert({"blocker_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"offender", {DealerFlagPriority::LOW_PRIORITY, {closeToBallFlag}}});

    return flagMap;  // DONT TOUCH.
}

uint8_t FreeKickThem::score(PlayEvaluator& playEvaluator) noexcept {
    /// List of all factors that combined results in an evaluation how good the play is.
    scoring = {{playEvaluator.getGlobalEvaluation(eval::FreeKickThemGameState), 1.0}};
    return (lastScore = playEvaluator.calculateScore(scoring)).value();  // DONT TOUCH.
}

void FreeKickThem::calculateInfoForRoles() noexcept {
    /// Function where are roles get their information, make sure not to compute roles twice.

    calculateInfoForKeeper();
    calculateInfoForBlockers();
    calculateInfoForDefenders();
    calculateInfoForOffenders();
}

void FreeKickThem::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void FreeKickThem::calculateInfoForBlockers() noexcept {
    // Factor that is multiplied by the minimal avoid ball distance.
    // We do not block enemy robots within this distance, since they are too close to the ball
    const double AVOID_BALL_DISTANCE_FACTOR = 1.5;

    auto enemyRobots = world->getWorld()->getThem();

    if (enemyRobots.empty()) {
        RTT_ERROR("There are no enemy robots, which are necessary for this play!")
        return;
    }
    // We cannot block enemy robots that are too close to the ball
    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),[&](const auto enemyRobot) -> bool {
            return enemyRobot->getDistanceToBall() <= AVOID_BALL_DISTANCE_FACTOR * control_constants::AVOID_BALL_DISTANCE;
        }), enemyRobots.end());

    for (auto &stpInfo : stpInfos) {
        if (stpInfo.first.find("blocker") != std::string::npos) {
            auto roleName = stpInfo.first;

            if (!enemyRobots.empty()) {
                // If there are enemy robots available, block the closest robot to the ball
                auto enemyToDefend = world->getWorld()->getRobotClosestToPoint(world->getWorld()->getBall().value()->getPos(), enemyRobots);

                enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),[&](const auto enemyRobot) -> bool {
                    return enemyToDefend && enemyRobot->getId() == enemyToDefend.value()->getId();
                }));

                stpInfos[roleName].setPositionToDefend(enemyToDefend ? enemyToDefend.value()->getPos() : field.getOurGoalCenter());
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
                stpInfos[roleName].setBlockDistance(BlockDistance::FAR);
            } else {
                // TODO: Improve default behaviour when there are no enemy robots to block
                stpInfos[roleName].setPositionToDefend(field.getOurGoalCenter());
                stpInfos[roleName].setEnemyRobot(world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::them));
                stpInfos[roleName].setBlockDistance(BlockDistance::HALFWAY);
            }
        }
    }
}

void FreeKickThem::calculateInfoForDefenders() noexcept {
    stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_0"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["defender_0"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_1"].setPositionToDefend(field.getOurTopGoalSide());
    stpInfos["defender_1"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
    stpInfos["defender_2"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);
}

void FreeKickThem::calculateInfoForOffenders() noexcept {
    stpInfos["offender"].setPositionToMoveTo(Vector2(field.getFieldLength() / 4, 0.0));
}

}  // namespace rtt::ai::stp::play
