//
// Created by agata on 29/06/2022.
//

#include "stp/plays/DrivingTest.h"

#include <stp/roles/passive/TestDriver.h>

#include "stp/roles/TestRole.h"
#include "stp/roles/active/BallPlacer.h"
#include "stp/roles/passive/BallAvoider.h"

namespace rtt::ai::stp {

DrivingTest::DrivingTest() : Play() {
    startPlayEvaluation.clear();  // DONT TOUCH.
    startPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    /// Evaluations that have to be true to allow the play to continue, otherwise the play will change. Plays can also end using the shouldEndPlay().
    keepPlayEvaluation.clear();  // DONT TOUCH.
    keepPlayEvaluation.emplace_back(eval::NormalPlayGameState);

    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{
        std::make_unique<role::TestDriver>(role::TestDriver("maniac_driver")),    std::make_unique<role::BallAvoider>(role::BallAvoider("keeper")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_0")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_1")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_2")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_3")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_4")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_5")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_6")), std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_7")),
        std::make_unique<role::BallAvoider>(role::BallAvoider("ball_avoider_8"))};
}

uint8_t DrivingTest::score(const rtt::world::Field& field) noexcept { return 0; }

Dealer::FlagMap DrivingTest::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;

    flagMap.insert({"keeper", {DealerFlagPriority::KEEPER, {}}});
    flagMap.insert({"maniac_driver", {DealerFlagPriority::REQUIRED, {}}});
    flagMap.insert({"ball_avoider_0", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_1", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_2", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_3", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_4", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_5", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_6", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_7", {DealerFlagPriority::LOW_PRIORITY, {}}});
    flagMap.insert({"ball_avoider_8", {DealerFlagPriority::LOW_PRIORITY, {}}});

    return flagMap;
}

void DrivingTest::calculateInfoForRoles() noexcept {
    stpInfos["keeper"].setPositionToMoveTo(Vector2(field.getOurGoalCenter() + Vector2(0.5, 0.0)));
    stpInfos["keeper"].setEnemyRobot(world->getWorld()->getRobotClosestToBall(world::them));

    stpInfos["maniac_driver"].setPositionToMoveTo(Vector2(0, field.getCenterY()));
}

const char* DrivingTest::getName() { return "Driving Test"; }

}  // namespace rtt::ai::stp
