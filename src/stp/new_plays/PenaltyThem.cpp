//
// Created by timovdk on 4/28/20.
//

#include <stp/invariants/BallCloseToUsInvariant.h>
#include <stp/invariants/BallMovesSlowInvariant.h>
#include <stp/new_plays/PenaltyThem.h>
#include <stp/new_roles/Formation.h>
#include <stp/new_roles/Keeper.h>

namespace rtt::ai::stp::play {

PenaltyThem::PenaltyThem() : Play() {
    // TODO: decide start invariants
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::BallCloseToUsInvariant>());

    // TODO: decide keep invariants
    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::BallMovesSlowInvariant>());

    roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::Keeper>(role::Keeper("keeper")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_0")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_1")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_2")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_3")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_4")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_5")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_6")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_7")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_8")),
                                                                                       std::make_unique<role::Formation>(role::Formation("formation_9"))};
}

uint8_t PenaltyThem::score(world_new::World *world) noexcept { return 111; }

Dealer::FlagMap PenaltyThem::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag closeToBallFlag(DealerFlagTitle::CLOSE_TO_BALL, DealerFlagPriority::HIGH_PRIORITY);
    Dealer::DealerFlag closeToTheirGoalFlag(DealerFlagTitle::CLOSE_TO_THEIR_GOAL, DealerFlagPriority::MEDIUM_PRIORITY);

    flagMap.insert({"keeper", {closeToBallFlag}});
    flagMap.insert({"formation_0", {closeToTheirGoalFlag}});
    flagMap.insert({"formation_1", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"formation_2", {closeToBallFlag}});
    flagMap.insert({"formation_3", {closeToTheirGoalFlag}});
    flagMap.insert({"formation_4", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"formation_5", {closeToBallFlag}});
    flagMap.insert({"formation_6", {closeToTheirGoalFlag}});
    flagMap.insert({"formation_7", {closeToTheirGoalFlag, closeToBallFlag}});
    flagMap.insert({"formation_8", {closeToBallFlag}});
    flagMap.insert({"formation_9", {closeToTheirGoalFlag}});

    return flagMap;
}

void PenaltyThem::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world_new::them));
    stpInfos["formation_0"].setPositionToMoveTo(Vector2{-2, -4});
    stpInfos["formation_1"].setPositionToMoveTo(Vector2{-2, -3});
    stpInfos["formation_2"].setPositionToMoveTo(Vector2{-2, -2});
    stpInfos["formation_3"].setPositionToMoveTo(Vector2{-2, -1});
    stpInfos["formation_4"].setPositionToMoveTo(Vector2{-2, 0});
    stpInfos["formation_5"].setPositionToMoveTo(Vector2{-2, 1});
    stpInfos["formation_6"].setPositionToMoveTo(Vector2{-2, 2});
    stpInfos["formation_7"].setPositionToMoveTo(Vector2{-2, 3});
    stpInfos["formation_8"].setPositionToMoveTo(Vector2{-2, 4});
    stpInfos["formation_9"].setPositionToMoveTo(Vector2{-1.5, 0});
}

bool PenaltyThem::shouldRoleSkipEndTactic() { return false; }

const char *PenaltyThem::getName() { return "Penalty Them Play"; }

}  // namespace rtt::ai::stp::play