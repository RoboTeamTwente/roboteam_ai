//
// Created by jordi on 01-05-20.
//

#include "include/roboteam_ai/stp/plays/referee_specific/TimeOut.h"

#include "stp/invariants/game_states/TimeOutGameStateInvariant.h"
#include "include/roboteam_ai/stp/roles/passive/Formation.h"

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

uint8_t TimeOut::score(world::World* world) noexcept { return 100; }

void TimeOut::calculateInfoForRoles() noexcept {
    const auto xPosition = -4 * control_constants::ROBOT_RADIUS;
    const double distanceToCenterLine = field.getFieldWidth() / 2 - 2*control_constants::ROBOT_RADIUS;
    const double yPosition = Constants::STD_TIMEOUT_TO_TOP() ? distanceToCenterLine: -distanceToCenterLine;

    const std::string formation = "time_out_";
    for(int i = 1; i <= 11; i++) {
        stpInfos[formation + std::to_string(i)].setPositionToMoveTo(Vector2(i * xPosition, yPosition));
    }
}

bool TimeOut::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap TimeOut::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"time_out_1", {notImportant}});
    flagMap.insert({"time_out_2", {notImportant}});
    flagMap.insert({"time_out_3", {notImportant}});
    flagMap.insert({"time_out_4", {notImportant}});
    flagMap.insert({"time_out_5", {notImportant}});
    flagMap.insert({"time_out_6", {notImportant}});
    flagMap.insert({"time_out_7", {notImportant}});
    flagMap.insert({"time_out_8", {notImportant}});
    flagMap.insert({"time_out_9", {notImportant}});
    flagMap.insert({"time_out_10", {notImportant}});
    flagMap.insert({"time_out_11", {notImportant}});

    return flagMap;
}

const char* TimeOut::getName() { return "Time Out"; }

}  // namespace rtt::ai::stp::play