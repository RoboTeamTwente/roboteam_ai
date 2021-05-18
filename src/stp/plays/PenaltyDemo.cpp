//
// Created by Jaro & Floris on 18-05-21.
//

#include <include/roboteam_ai/stp/roles/DemoKeeper.h>
#include "include/roboteam_ai/stp/plays/PenaltyDemo.h"
#include "stp/roles/Halt.h"
#include "stp/roles/PenaltyKeeper.h"

namespace rtt::ai::stp::play {

    PenaltyDemo::PenaltyDemo() : Play() {
        startPlayInvariants.clear();

        keepPlayInvariants.clear();

        roles = std::array<std::unique_ptr<Role>, stp::control_constants::MAX_ROBOT_COUNT>{std::make_unique<role::DemoKeeper>(role::DemoKeeper("keeper")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_0")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_1")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_2")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_3")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_4")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_5")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_6")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_7")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_8")),
                                                                                           std::make_unique<role::Halt>(role::Halt("halt_9"))};
    }

    uint8_t PenaltyDemo::score(world::World *world) noexcept { return 0; }

    Dealer::FlagMap PenaltyDemo::decideRoleFlags() const noexcept {
        Dealer::FlagMap flagMap;
        Dealer::DealerFlag keeperFlag(DealerFlagTitle::KEEPER, DealerFlagPriority::KEEPER);
        Dealer::DealerFlag notImportant(DealerFlagTitle::NOT_IMPORTANT, DealerFlagPriority::LOW_PRIORITY);

        flagMap.insert({"keeper", {keeperFlag}});
        flagMap.insert({"halt_0", {notImportant}});
        flagMap.insert({"halt_1", {notImportant}});
        flagMap.insert({"halt_2", {notImportant}});
        flagMap.insert({"halt_3", {notImportant}});
        flagMap.insert({"halt_4", {notImportant}});
        flagMap.insert({"halt_5", {notImportant}});
        flagMap.insert({"halt_6", {notImportant}});
        flagMap.insert({"halt_7", {notImportant}});
        flagMap.insert({"halt_8", {notImportant}});
        flagMap.insert({"halt_9", {notImportant}});

        return flagMap;
    }

    void PenaltyDemo::calculateInfoForRoles() noexcept {
        stpInfos["keeper"].setPositionToMoveTo(field.getOurGoalCenter());
        stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));
    }

    bool PenaltyDemo::shouldRoleSkipEndTactic() { return false; }

    const char *PenaltyDemo::getName() { return "Penalty Demo"; }

}  // namespace rtt::ai::stp::play