//
// Created by mrlukasbos on 21-5-19.
//


#include <gtest/gtest.h>
#include <roboteam_ai/src/control/positionControllers/NumTreePosControl.h>
#include <roboteam_ai/src/utilities/GameStateManager.hpp>

namespace rtt {
namespace ai {
namespace control {

TEST(NumTreePosControlTest, it_obeys_the_referee) {
    NumTreePosControl gtp;

    /*
     * Set the gamestate to normal play such that we are not allowed to move in the defense area
     * even if we want to. Then change the state to ballplacement where it is allowed and then it should be possible.
     */
    GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
    gtp.setCanMoveInDefenseArea(false);
    EXPECT_FALSE(gtp.getCanMoveInDefenseArea());
    gtp.setCanMoveInDefenseArea(true);
    EXPECT_FALSE(gtp.getCanMoveInDefenseArea());
    GameStateManager::forceNewGameState(RefCommand::BALL_PLACEMENT_US);
    gtp.setCanMoveInDefenseArea(true);
    EXPECT_TRUE(gtp.getCanMoveInDefenseArea());

    gtp.setCanMoveOutOfField(false);
    EXPECT_FALSE(gtp.getCanMoveOutOfField());
    gtp.setCanMoveOutOfField(true);
    EXPECT_TRUE(gtp.getCanMoveOutOfField());

    /*
     * Set the gamestate to normal play such that we are not allowed to get close to the ball
     * even if we want to. Then change the state to ballplacement where it is allowed and then it should be possible.
     */
    GameStateManager::forceNewGameState(RefCommand::BALL_PLACEMENT_THEM);
    gtp.setAvoidBallDistance(0.5);
    EXPECT_EQ(gtp.getAvoidBallDistance(), 0.8);

    GameStateManager::forceNewGameState(RefCommand::BALL_PLACEMENT_US);
    EXPECT_EQ(gtp.getAvoidBallDistance(), 0.5);
}

} // control
} // ai
} // rtt
