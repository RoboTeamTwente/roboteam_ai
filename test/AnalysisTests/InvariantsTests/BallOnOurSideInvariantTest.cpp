//
// Created by jessevw on 20.12.19.
//

#include <analysis/DecisionMaker.h>
#include <gtest/gtest.h>
#include <roboteam_utils/Vector2.h>
#include <utilities/RobotDealer.h>
#include <world/Field.h>
#include <world/World.h>

#include "test/helpers/WorldHelper.h"
namespace rtt::ai::analysis {

    TEST(BallOnOurSideInvariantTest, ball_on_our_side_is_valid_test) {
        /// TODO: Fix this test
        //        auto location = Vector2(0,0);
        //        auto ball = testhelpers::WorldHelper::generateBallAtLocation(location);
        //
        //        const proto::World world;
        //        world.set_allocated_ball(ball);
        //
        //        auto testWorld = std::make_unique<world::World>(world);
        //        auto testField = std::make_unique<world::Field>();

        EXPECT_TRUE(true);
    }

}  // namespace rtt::ai::analysis
