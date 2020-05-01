//
// Created by jordi on 01-05-20.
//

#include "stp/new_plays/TimeOut.h"
#include "stp/new_roles/Formation.h"
#include "stp/invariants/game_states/TimeOutGameStateInvariant.h"
#include "roboteam_utils/Hungarian.h"

namespace rtt::ai::stp::play {

TimeOut::TimeOut() : Play() {
    startPlayInvariants.clear();
    startPlayInvariants.emplace_back(std::make_unique<invariant::TimeOutGameStateInvariant>());

    keepPlayInvariants.clear();
    keepPlayInvariants.emplace_back(std::make_unique<invariant::TimeOutGameStateInvariant>());

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
            std::make_unique<role::Formation>(role::Formation("time_out_1")), std::make_unique<role::Formation>(role::Formation("time_out_2")),
            std::make_unique<role::Formation>(role::Formation("time_out_3")), std::make_unique<role::Formation>(role::Formation("time_out_4")),
            std::make_unique<role::Formation>(role::Formation("time_out_5")), std::make_unique<role::Formation>(role::Formation("time_out_6")),
            std::make_unique<role::Formation>(role::Formation("time_out_7")), std::make_unique<role::Formation>(role::Formation("time_out_8")),
            std::make_unique<role::Formation>(role::Formation("time_out_9")), std::make_unique<role::Formation>(role::Formation("time_out_10")),
            std::make_unique<role::Formation>(role::Formation("time_out_11"))};
}

uint8_t TimeOut::score(world_new::World* world) noexcept { return 100; }

void TimeOut::calculateInfoForRoles() noexcept {
    auto xPosition = -4 * control_constants::ROBOT_RADIUS;
    auto yPosition = Constants::STD_TIMEOUT_TO_TOP() ? field.getFieldWidth() / 2 : -field.getFieldWidth() / 2;

    std::unordered_map<int, Vector2> currentPositions;

    for (const auto& robot : world->getWorld()->getUs()) {
        currentPositions.insert({robot->getId(), robot->getPos()});
    }

    std::vector<Vector2> timeOutPositions;
    timeOutPositions.reserve(stpInfos.size());

    for (int i = 0; i < stpInfos.size(); i++) {
        timeOutPositions.emplace_back(Vector2(xPosition - i * 4 * control_constants::ROBOT_RADIUS, yPosition));
    }

    // Minimize the distance between the robots and the time-out positions
    auto optimizedTimeOutPositions = Hungarian::getOptimalPairsIdentified(currentPositions, timeOutPositions);

    for (auto& stpInfo : stpInfos) {
        stpInfo.second.setPositionToMoveTo(optimizedTimeOutPositions[stpInfo.second.getRobot().value()->getId()]);
    }

}

bool TimeOut::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap TimeOut::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"time_out_1", {}});
    flagMap.insert({"time_out_2", {}});
    flagMap.insert({"time_out_3", {}});
    flagMap.insert({"time_out_4", {}});
    flagMap.insert({"time_out_5", {}});
    flagMap.insert({"time_out_6", {}});
    flagMap.insert({"time_out_7", {}});
    flagMap.insert({"time_out_8", {}});
    flagMap.insert({"time_out_9", {}});
    flagMap.insert({"time_out_10", {}});
    flagMap.insert({"time_out_11", {}});

    return flagMap;
}

const char *TimeOut::getName() {
    return "Time Out";
}

}  // namespace rtt::ai::stp::play