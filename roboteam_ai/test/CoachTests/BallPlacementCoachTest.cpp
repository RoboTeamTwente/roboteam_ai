//
// Created by mrlukasbos on 20-3-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/coach/Ballplacement.h>
#include <roboteam_ai/src/interface/InterfaceValues.h>
#include <roboteam_ai/src/control/ControlUtils.h>
#include <roboteam_ai/src/world/World.h>

TEST(CoachTest, it_handles_ballplacement_positions) {
    rtt::ai::interface::InterfaceValues::setBallPlacementTarget({2.3, 0.3});

    auto ballplacementTarget = rtt::ai::coach::g_ballPlacement.getBallPlacementPos();
    EXPECT_EQ(ballplacementTarget.x, 2.3);
    EXPECT_EQ(ballplacementTarget.y, 0.3);

    EXPECT_FLOAT_EQ(ballplacementTarget.dist(rtt::ai::coach::g_ballPlacement.getBallPlacementAfterPos(0.2)), rtt::ai::Constants::BP_MOVE_BACK_DIST());

    roboteam_msgs::World worldMsg;
    worldMsg.ball.pos = Vector2(0, 0);
    worldMsg.ball.visible = 1;
    worldMsg.ball.existence = 9999;
    rtt::ai::world::world->setWorld(worldMsg);

    EXPECT_FLOAT_EQ(Vector2(rtt::ai::world::world->getBall()->pos).dist(rtt::ai::coach::g_ballPlacement.getBallPlacementBeforePos(rtt::ai::world::world->getBall()->pos)), rtt::ai::Constants::BP_MOVE_TOWARDS_DIST());
}