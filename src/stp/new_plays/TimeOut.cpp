//
// Created by jordi on 01-05-20.
//

#include "stp/new_plays/TimeOut.h"

#include "stp/invariants/game_states/TimeOutGameStateInvariant.h"
#include "stp/new_roles/Formation.h"

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
    const auto xPosition = -4 * control_constants::ROBOT_RADIUS;
    const auto yPosition = Constants::STD_TIMEOUT_TO_TOP() ? field.getFieldWidth() / 2.2 : -field.getFieldWidth() / 2.2;

    stpInfos["time_out_1"].setPositionToMoveTo(Vector2(xPosition, yPosition));
    stpInfos["time_out_2"].setPositionToMoveTo(Vector2(xPosition - 4 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_3"].setPositionToMoveTo(Vector2(xPosition - 8 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_4"].setPositionToMoveTo(Vector2(xPosition - 12 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_5"].setPositionToMoveTo(Vector2(xPosition - 16 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_6"].setPositionToMoveTo(Vector2(xPosition - 20 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_7"].setPositionToMoveTo(Vector2(xPosition - 24 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_8"].setPositionToMoveTo(Vector2(xPosition - 28 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_9"].setPositionToMoveTo(Vector2(xPosition - 32 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_10"].setPositionToMoveTo(Vector2(xPosition - 36 * control_constants::ROBOT_RADIUS, yPosition));
    stpInfos["time_out_11"].setPositionToMoveTo(Vector2(xPosition - 40 * control_constants::ROBOT_RADIUS, yPosition));
}

bool TimeOut::shouldRoleSkipEndTactic() { return false; }

Dealer::FlagMap TimeOut::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    Dealer::DealerFlag not_important(DealerFlagTitle::ROBOT_TYPE_50W, DealerFlagPriority::LOW_PRIORITY);

    flagMap.insert({"time_out_1", {not_important}});
    flagMap.insert({"time_out_2", {not_important}});
    flagMap.insert({"time_out_3", {not_important}});
    flagMap.insert({"time_out_4", {not_important}});
    flagMap.insert({"time_out_5", {not_important}});
    flagMap.insert({"time_out_6", {not_important}});
//    flagMap.insert({"time_out_7", {not_important}});
//    flagMap.insert({"time_out_8", {not_important}});
//    flagMap.insert({"time_out_9", {not_important}});
//    flagMap.insert({"time_out_10", {not_important}});
//    flagMap.insert({"time_out_11", {not_important}});

    return flagMap;
}

const char* TimeOut::getName() { return "Time Out"; }

}  // namespace rtt::ai::stp::play