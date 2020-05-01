//
// Created by jordi on 30-04-20.
//

#include "stp/new_plays/KickOffThemPrepare.h"
#include "stp/new_roles/Formation.h"
#include "stp/invariants/game_states/KickOffThemPrepareGameStateInvariant.h"
#include "roboteam_utils/Hungarian.h"

namespace rtt::ai::stp::play {

KickOffThemPrepare::KickOffThemPrepare() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::KickOffThemPrepareGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::KickOffThemPrepareGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::Formation>(role::Formation("keeper")),      std::make_unique<role::Formation>(role::Formation("formation_1")),
            std::make_unique<role::Formation>(role::Formation("formation_2")), std::make_unique<role::Formation>(role::Formation("formation_3")),
            std::make_unique<role::Formation>(role::Formation("formation_4")), std::make_unique<role::Formation>(role::Formation("formation_5")),
            std::make_unique<role::Formation>(role::Formation("formation_6")), std::make_unique<role::Formation>(role::Formation("formation_7")),
            std::make_unique<role::Formation>(role::Formation("formation_8")), std::make_unique<role::Formation>(role::Formation("formation_9")),
            std::make_unique<role::Formation>(role::Formation("formation_10"))};
}

uint8_t KickOffThemPrepare::score(world_new::World* world) noexcept { return 100; }

void KickOffThemPrepare::calculateInfoForRoles() noexcept {
    auto width = field.getFieldWidth();
    auto length = field.getFieldLength();

    // Keeper
    if (stpInfos.find("keeper") != stpInfos.end()) {
        stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
    }

    // Positions of the kick off them formation which will be dealt to the Formation roles in order
    std::vector<Vector2> formationPositions = {
            Vector2(-length/4, width/8), Vector2(-length/4, -width/8), Vector2(-length/8, width/4), Vector2(-length/8, -width/4),
            Vector2(-length*3/8, 0.0), Vector2(-length*3/8, width/5), Vector2(-length*3/8, -width/5),
            Vector2(-length/4, width/3), Vector2(-length/4, -width/3), Vector2(-length/8, 0.0)
    };

    int numberOfFormationRole = 0;
    std::unordered_map<int, Vector2> currentPositions;
    std::vector<Vector2> usedFormationPositions;

    for (auto& stpInfo : stpInfos) {
        if (stpInfo.first != "keeper") {
            auto robot = stpInfo.second.getRobot().value();
            currentPositions.insert({robot->getId(), robot->getPos()});
            usedFormationPositions.emplace_back(formationPositions[numberOfFormationRole]);
            numberOfFormationRole++;
        }
    }

    // Minimize the distance between the robots and the used formation positions
    auto optimizedFormationPositions = Hungarian::getOptimalPairsIdentified(currentPositions, usedFormationPositions);

    for (auto& stpInfo : stpInfos) {
        if (stpInfo.first != "keeper") {
            stpInfo.second.setPositionToMoveTo(optimizedFormationPositions[stpInfo.second.getRobot().value()->getId()]);
        }
    }
}

bool KickOffThemPrepare::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap KickOffThemPrepare::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::REQUIRED);

    flagMap.insert({"keeper", {keeperFlag}});
    flagMap.insert({"formation_1", {}});
    flagMap.insert({"formation_2", {}});
    flagMap.insert({"formation_3", {}});
    flagMap.insert({"formation_4", {}});
    flagMap.insert({"formation_5", {}});
    flagMap.insert({"formation_6", {}});
    flagMap.insert({"formation_7", {}});
    flagMap.insert({"formation_8", {}});
    flagMap.insert({"formation_9", {}});
    flagMap.insert({"formation_10", {}});

    return flagMap;
}

const char *KickOffThemPrepare::getName() {
    return "Kick Off Them Prepare";
}

}  // namespace rtt::ai::stp::play