//
// Created by jordi on 27-03-20.
//

#include "include/roboteam_ai/stp/plays/defensive/DefendShot.h"

#include "include/roboteam_ai/stp/roles/passive/Defender.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"
#include "include/roboteam_ai/stp/roles/passive/Harasser.h"
#include "stp/roles/Keeper.h"
#include "stp/computations/PositionComputations.h"

namespace rtt::ai::stp::play {

    DefendShot::DefendShot() : Play() {
        startPlayEvaluation.clear();
        startPlayEvaluation.emplace_back(eval::NormalPlayGameState);
        startPlayEvaluation.emplace_back(eval::BallCloseToThem);

        keepPlayEvaluation.clear();
        keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);
        keepPlayEvaluation.emplace_back(eval::BallShotOrCloseToThem);

        roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{
                std::make_unique<role::Keeper>(role::Keeper("keeper")),
                std::make_unique<role::Defender>(role::Defender("defender_1")),
                std::make_unique<role::Defender>(role::Defender("defender_2")),
                std::make_unique<role::Defender>(role::Defender("defender_3")),
                std::make_unique<role::Defender>(role::Defender("defender_4")),
                std::make_unique<role::Defender>(role::Defender("defender_5")),
                std::make_unique<role::Harasser>(role::Harasser("harasser")),
                std::make_unique<role::Formation>(role::Formation("midfielder_1")),
                std::make_unique<role::Formation>(role::Formation("midfielder_2")),
                std::make_unique<role::Formation>(role::Formation("offender_1")),
                std::make_unique<role::Formation>(role::Formation("offender_2"))};
    }

    uint8_t DefendShot::score(PlayEvaluator &playEvaluator) noexcept { return 100; }

    Dealer::FlagMap DefendShot::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;

        Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
        Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::LOW_PRIORITY);
        Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
        flagMap.insert({"harasser", {DealerFlagPriority::REQUIRED, {closeToBallFlag}}});
        flagMap.insert({"defender_1", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"defender_2", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"defender_3", {DealerFlagPriority::HIGH_PRIORITY, {closeToOurGoalFlag}}});
        flagMap.insert({"defender_4", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"defender_5", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"midfielder_1", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"midfielder_2", {DealerFlagPriority::LOW_PRIORITY, {notImportant}}});
        flagMap.insert({"offender_1", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToTheirGoalFlag}}});
        flagMap.insert({"offender_2", {DealerFlagPriority::MEDIUM_PRIORITY, {closeToTheirGoalFlag}}});

        return flagMap;
    }

    void DefendShot::calculateInfoForRoles() noexcept {
        calculateInfoForDefenders();
        calculateInfoForHarassers();
        calculateInfoForKeeper();
        calculateInfoForMidfielders();
        calculateInfoForOffenders();
    }

// TODO-Max move to Tactics
    void DefendShot::calculateInfoForDefenders() noexcept {
        /// Defenders
        auto enemyRobots = world->getWorld()->getThem();

        stpInfos["defender_0"].setPositionToDefend(field.getOurGoalCenter());
        stpInfos["defender_0"].setEnemyRobot(enemyRobots[0]);
        stpInfos["defender_0"].setBlockDistance(BlockDistance::HALFWAY);

        stpInfos["defender_1"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_1"].setEnemyRobot(enemyRobots[1]);
        stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

        stpInfos["defender_2"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_2"].setEnemyRobot(enemyRobots[2]);
        stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

        stpInfos["defender_3"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_3"].setEnemyRobot(enemyRobots[3]);
        stpInfos["defender_3"].setBlockDistance(BlockDistance::HALFWAY);

        stpInfos["defender_4"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_4"].setEnemyRobot(enemyRobots[4]);
        stpInfos["defender_4"].setBlockDistance(BlockDistance::HALFWAY);

        stpInfos["defender_5"].setPositionToDefend(field.getOurBottomGoalSide());
        stpInfos["defender_5"].setEnemyRobot(enemyRobots[5]);
        stpInfos["defender_5"].setBlockDistance(BlockDistance::HALFWAY);

//        auto enemyAttacker = world->getWorld()->getRobotClosestToBall(world::them);
//
//        if (enemyRobots.empty()) {
//            RTT_ERROR("There are no enemy robots, which are necessary for this play!")
//            return;
//        }
//
//        enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
//                                         [&](const auto enemyRobot) -> bool {
//                                             return enemyAttacker && enemyRobot->getId() ==
//                                                                     enemyAttacker.value()->getId();
//                                         }));
//
//        auto enemyClosestToGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);
//
//        auto secondEnemyClosestToGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(),
//                                                                                  enemyRobots);
//
//        stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
//        stpInfos["defender_1"].setEnemyRobot(enemyAttacker);
//        stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);
//
//        stpInfos["defender_2"].setPositionToDefend(field.getOurGoalCenter());
//        stpInfos["defender_2"].setEnemyRobot(enemyClosestToGoal);
//        stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);
//
//        if (enemyClosestToGoal)
//            stpInfos["defender_3"].setPositionToDefend(enemyClosestToGoal.value()->getPos());
//        else
//            stpInfos["defender_3"].setPositionToDefend(std::nullopt);
//        stpInfos["defender_3"].setEnemyRobot(enemyAttacker);
//        stpInfos["defender_3"].setBlockDistance(BlockDistance::HALFWAY);
//
//        stpInfos["defender_4"].setPositionToDefend(field.getOurGoalCenter());
//        stpInfos["defender_4"].setEnemyRobot(secondEnemyClosestToGoal);
//        stpInfos["defender_4"].setBlockDistance(BlockDistance::HALFWAY);
//
//        if (enemyClosestToGoal)
//            stpInfos["defender_5"].setPositionToDefend(secondEnemyClosestToGoal.value()->getPos());
//        else
//            stpInfos["defender_5"].setPositionToDefend(std::nullopt);
//        stpInfos["defender_5"].setEnemyRobot(enemyAttacker);
//        stpInfos["defender_5"].setBlockDistance(BlockDistance::HALFWAY);

        // When the ball moves, one defender tries to intercept the ball
        auto closestBotUs = world->getWorld()->getRobotClosestToBall(world::us);
        auto closestBotThem = world->getWorld()->getRobotClosestToBall(world::them);
        for (auto &role : roles) {
            auto roleName = role->getName();
            if (closestBotUs && closestBotThem && roleName.find("defender") != std::string::npos) {
                // TODO: Improve choice of intercept robot based on trajectory and intercept position
                if (stpInfos[roleName].getRobot() &&
                    stpInfos[roleName].getRobot().value()->getId() == closestBotUs.value()->getId() &&
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

    void DefendShot::calculateInfoForHarassers() noexcept {
        stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    }

    void DefendShot::calculateInfoForKeeper() noexcept {
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
        stpInfos["keeper"].setPositionToShootAt(
                world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), world::us).value()->getPos());
    }

    void DefendShot::calculateInfoForMidfielders() noexcept {
        auto ballPos = world->getWorld()->getBall().value()->getPos();

        stpInfos["midfielder_1"].setAngle((ballPos - stpInfos["midfielder_1"].getRobot()->get()->getPos()).angle());
        stpInfos["midfielder_1"].setPositionToMoveTo(PositionComputations::getPosition(
                stpInfos["midfielder_1"].getPositionToMoveTo(), gen::gridMidFieldBot, gen::SafePosition, field, world));

        stpInfos["midfielder_2"].setAngle((ballPos - stpInfos["midfielder_2"].getRobot()->get()->getPos()).angle());
        stpInfos["midfielder_2"].setPositionToMoveTo(PositionComputations::getPosition(
                stpInfos["midfielder_2"].getPositionToMoveTo(), gen::gridMidFieldTop, gen::SafePosition, field, world));
    }

    void DefendShot::calculateInfoForOffenders() noexcept {
        stpInfos["offender_1"].setPositionToMoveTo(PositionComputations::getPosition(
                stpInfos["offender_1"].getPositionToMoveTo(), gen::gridRightBot, gen::SafePosition, field, world));
        stpInfos["offender_2"].setPositionToMoveTo(PositionComputations::getPosition(
                stpInfos["offender_2"].getPositionToMoveTo(), gen::gridRightTop, gen::SafePosition, field, world));
    }

    const char *DefendShot::getName() { return "Defend Shot"; }

}  // namespace rtt::ai::stp::play
