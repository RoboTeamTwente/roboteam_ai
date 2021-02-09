//
// Created by jordi on 27-03-20.
//

#include "include/roboteam_ai/stp/plays/defensive/DefendShot.h"

#include "stp/invariants/BallCloseToThemInvariant.h"
#include "stp/invariants/BallShotOrCloseToThemInvariant.h"
#include "stp/invariants/game_states/NormalPlayGameStateInvariant.h"
#include "include/roboteam_ai/stp/roles/passive/Defender.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"
#include "include/roboteam_ai/stp/roles/passive/Harasser.h"
#include "stp/roles/Keeper.h"

namespace rtt::ai::stp::play {

DefendShot::DefendShot() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToThemInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::NormalPlayGameStateInvariant>());
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallShotOrCloseToThemInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
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

uint8_t DefendShot::score(world::World *world) noexcept { return 100; }

Dealer::FlagMap DefendShot::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToOurGoalFlag(DealerFlagTitle::CLOSE_TO_OUR_GOAL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::LOW_PRIORITY);
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"defender_1", {closeToOurGoalFlag}});
    flagMap.insert({"defender_2", {closeToOurGoalFlag}});
    flagMap.insert({"defender_3", {closeToOurGoalFlag}});
    flagMap.insert({"defender_4", {notImportant}});
    flagMap.insert({"defender_5", {notImportant}});
    flagMap.insert({"harasser", {closeToBallFlag}});
    flagMap.insert({"midfielder_1", {notImportant}});
    flagMap.insert({"midfielder_2", {notImportant}});
    flagMap.insert({"offender_1", {closeToTheirGoalFlag}});
    flagMap.insert({"offender_2", {closeToTheirGoalFlag}});

    return flagMap;
}

void DefendShot::calculateInfoForRoles() noexcept {
    calculateInfoForDefenders();
    calculateInfoForHarassers();
    calculateInfoForKeeper();
    calculateInfoForMidfielders();
    calculateInfoForOffenders();
}

void DefendShot::calculateInfoForDefenders() noexcept {
    auto enemyRobots = world->getWorld()->getThem();

    auto enemyAttacker = world->getWorld()->getRobotClosestToBall(world::them);

    if (enemyRobots.empty()) {
        RTT_ERROR("There are no enemy robots, which are necessary for this play!")
        return;
    }

    enemyRobots.erase(std::remove_if(enemyRobots.begin(), enemyRobots.end(),
                                     [&](const auto enemyRobot) -> bool { return enemyAttacker && enemyRobot->getId() == enemyAttacker.value()->getId(); }));

    auto enemyClosestToGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    auto secondEnemyClosestToGoal = world->getWorld()->getRobotClosestToPoint(field.getOurGoalCenter(), enemyRobots);

    stpInfos["defender_1"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_1"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_1"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_2"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_2"].setEnemyRobot(enemyClosestToGoal);
    stpInfos["defender_2"].setBlockDistance(BlockDistance::HALFWAY);

    if (enemyClosestToGoal)
        stpInfos["defender_3"].setPositionToDefend(enemyClosestToGoal.value()->getPos());
    else
        stpInfos["defender_3"].setPositionToDefend(std::nullopt);
    stpInfos["defender_3"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_3"].setBlockDistance(BlockDistance::HALFWAY);

    stpInfos["defender_4"].setPositionToDefend(field.getOurGoalCenter());
    stpInfos["defender_4"].setEnemyRobot(secondEnemyClosestToGoal);
    stpInfos["defender_4"].setBlockDistance(BlockDistance::HALFWAY);

    if (enemyClosestToGoal)
        stpInfos["defender_5"].setPositionToDefend(secondEnemyClosestToGoal.value()->getPos());
    else
        stpInfos["defender_5"].setPositionToDefend(std::nullopt);
    stpInfos["defender_5"].setEnemyRobot(enemyAttacker);
    stpInfos["defender_5"].setBlockDistance(BlockDistance::HALFWAY);

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

void DefendShot::calculateInfoForHarassers() noexcept { stpInfos["harasser"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them)); }

void DefendShot::calculateInfoForKeeper() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    stpInfos["keeper"].setPositionToShootAt(Vector2());
}

void DefendShot::calculateInfoForMidfielders() noexcept {
    auto ball = world->getWorld()->getBall().value();

    if (stpInfos["midfielder_1"].getRobot() && stpInfos["midfielder_2"].getRobot()) {
        stpInfos["midfielder_1"].setAngle((ball->getPos() - stpInfos["midfielder_1"].getRobot()->get()->getPos()).angle());
        stpInfos["midfielder_2"].setAngle((ball->getPos() - stpInfos["midfielder_2"].getRobot()->get()->getPos()).angle());
    }

    auto searchGridLeft = Grid(0, 0, 1.5, 1.5, 3, 3);
    auto searchGridRight = Grid(0, -1.5, 1.5, 1.5, 3, 3);

    stpInfos["midfielder_1"].setPositionToMoveTo(control::ControlUtils::determineMidfielderPosition(searchGridRight, field, world));
    stpInfos["midfielder_2"].setPositionToMoveTo(control::ControlUtils::determineMidfielderPosition(searchGridLeft, field, world));
}

void DefendShot::calculateInfoForOffenders() noexcept {
    auto length = field.getFieldLength();
    auto width = field.getFieldWidth();

    stpInfos["offender_1"].setPositionToMoveTo(Vector2(length / 4, width / 6));
    stpInfos["offender_2"].setPositionToMoveTo(Vector2(length / 4, -width / 6));
}

bool DefendShot::shouldRoleSkipEndTactic() { return false; }

const char *DefendShot::getName() { return "Defend Shot"; }

}  // namespace rtt::ai::stp::play
