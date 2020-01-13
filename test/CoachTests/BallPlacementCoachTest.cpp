//
// Created by mrlukasbos on 20-3-19.
//

#include <gtest/gtest.h>
#include <coach/BallplacementCoach.h>
#include <interface/api/Output.h>
#include <control/ControlUtils.h>
#include "world_old/World.h"

TEST(CoachTest, it_handles_ballplacement_positions) {
    rtt::ai::interface::Output::setMarkerPosition({2.3, 0.3});

    auto ballplacementTarget = rtt::ai::coach::g_ballPlacement.getBallPlacementPos();
    EXPECT_EQ(ballplacementTarget.x, 2.3);
    EXPECT_EQ(ballplacementTarget.y, 0.3);

    roboteam_msgs::World worldMsg;
    worldMsg.ball.pos = Vector2(0, 0);
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 9999;
    rtt::ai::world::world->updateWorld(worldMsg);

    EXPECT_FLOAT_EQ(Vector2(rtt::ai::world::world->getBall()->pos).dist(rtt::ai::coach::g_ballPlacement.getBallPlacementBeforePos(rtt::ai::world::world->getBall()->pos)), rtt::ai::Constants::BP_MOVE_TOWARDS_DIST());
}