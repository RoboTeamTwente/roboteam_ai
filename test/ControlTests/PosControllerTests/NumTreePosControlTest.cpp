//
// Created by mrlukasbos on 21-5-19.
//

#include <control/numtrees/NumTreePosControl.h>
#include <gtest/gtest.h>
#include <test/helpers/WorldHelper.h>
#include <utilities/RobotDealer.h>

#include <utilities/GameStateManager.hpp>

namespace rtt {
namespace ai {
namespace control {

TEST(NumTreePosControlTest, it_obeys_the_referee) {
    NumTreePosControl gtp;
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 8;
    field.field_length = 12;
    roboteam_msgs::World worldmsg = testhelpers::WorldHelper::getWorldMsg(3, 0, false, field);
    world::world->updateWorld(worldmsg);
    /*
     * Set the gamestate to normal play such that we are not allowed to move in the defense area
     * even if we want to. Then change the state to ballplacement where it is allowed and then it should be possible.
     */

    robotDealer::RobotDealer::setKeeperID(0);

    GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
    gtp.setCanMoveInDefenseArea(false);
    EXPECT_FALSE(gtp.getCanMoveInDefenseArea(1));
    EXPECT_FALSE(gtp.getCanMoveInDefenseArea(0));  // the keeper can always move in the defense area

    gtp.setCanMoveInDefenseArea(true);
    EXPECT_FALSE(gtp.getCanMoveInDefenseArea(1));
    EXPECT_TRUE(gtp.getCanMoveInDefenseArea(0));  // the keeper can always move in the defense area
    GameStateManager::forceNewGameState(RefCommand::BALL_PLACEMENT_US);
    gtp.setCanMoveInDefenseArea(true);
    EXPECT_TRUE(gtp.getCanMoveInDefenseArea(1));

    gtp.setCanMoveOutOfField(false);
    EXPECT_FALSE(gtp.getCanMoveOutOfField(1));
    gtp.setCanMoveOutOfField(true);
    EXPECT_TRUE(gtp.getCanMoveOutOfField(1));

    /*
     * Set the gamestate to normal play such that we are not allowed to get close to the ball
     * even if we want to. Then change the state to ballplacement where it is allowed and then it should be possible.
     */
    GameStateManager::forceNewGameState(RefCommand::BALL_PLACEMENT_THEM);
    gtp.setAvoidBallDistance(0.5);
    EXPECT_EQ(gtp.getAvoidBallDistance(), 0.8);

    gtp.setAvoidBallDistance(1.0);
    EXPECT_EQ(gtp.getAvoidBallDistance(), 1.0);

    GameStateManager::forceNewGameState(RefCommand::BALL_PLACEMENT_US);
    gtp.setAvoidBallDistance(0.5);
    EXPECT_EQ(gtp.getAvoidBallDistance(), 0.5);
}

}  // namespace control
}  // namespace ai
}  // namespace rtt
