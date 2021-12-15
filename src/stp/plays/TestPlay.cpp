//
// Created by timovdk on 3/10/20.
//

/// This play is for testing purposes, to use it, be sure to uncomment in in ApplicationManager.cpp's start function!

#include "stp/plays/TestPlay.h"

#include "stp/roles/TestRole.h"
#include "stp/roles/passive/Halt.h"

std::queue<int> ballx;
std::queue<int> bally;
std::queue<Vector2> vel;
int velocity = 0;
namespace rtt::ai::stp {

TestPlay::TestPlay() : Play() {
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Role creation, the names should be unique. The names are used in the stpInfos-map.
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<TestRole>("role_0"), std::make_unique<role::Halt>(role::Halt("formation_1")), std::make_unique<role::Halt>(role::Halt("formation_2")),
        std::make_unique<role::Halt>(role::Halt("formation_3")), std::make_unique<role::Halt>(role::Halt("formation_4")),std::make_unique<role::Halt>(role::Halt("formation_5")),
        std::make_unique<role::Halt>(role::Halt("formation_6")), std::make_unique<role::Halt>(role::Halt("formation_7")), std::make_unique<role::Halt>(role::Halt("formation_8")),
        std::make_unique<role::Halt>(role::Halt("formation_9")),std::make_unique<role::Halt>(role::Halt("formation_10"))};


}

uint8_t TestPlay::score(PlayEvaluator &playEvaluator) noexcept { return 0; }

Dealer::FlagMap TestPlay::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"role_0", {DealerFlagPriority::REQUIRED, {}}});
    flagMap.insert({"role_1", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_2", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_3", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_4", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_5", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_6", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_7", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_8", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_9", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});
    flagMap.insert({"role_10", {DealerFlagPriority::MEDIUM_PRIORITY, {}}});

    return flagMap;
}

void TestPlay::calculateInfoForRoles() noexcept {
    /// Function where are roles get their information, make sure not to compute roles twice.

    // TODO: the shoot position might need to change
    stpInfos["role_0"].setPositionToMoveTo(world->getWorld()->getBall()->get()->getPos() - Vector2{0.04, 0.0});
    stpInfos["role_0"].setPositionToShootAt(world->getField()->getTheirGoalCenter());
    stpInfos["role_0"].setShotType(ShotType::MAX);

    stpInfos["role_0"].setKickChipVelocity(6.5);
    stpInfos["role_0"].setDribblerSpeed(0);
    double x = pow(world->getWorld()->getBall()->get()->getVelocity().x,2);
    double y = pow(world->getWorld()->getBall()->get()->getVelocity().y,2);

    double new_velocity = sqrt(x + y);

    double start = 0;

    double end = start - world->getWorld()->getBall()->get()->getPos().y;


    RTT_DEBUG(new_velocity);

}


const char *TestPlay::getName() { return "Test Play"; }

}  // namespace rtt::ai::stp
