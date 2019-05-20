//
// Created by mrlukasbos on 20-5-19.
//


#include <gtest/gtest.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include "../../src/world/BallPossession.h"

namespace rtt {
namespace ai {

TEST(BallPossessionTest, stateAsString) {
    rtt::ai::BallPossession bp;
    EXPECT_EQ(bp.stateAsString(rtt::ai::BallPossession::LOOSEBALL), "LOOSE");
    EXPECT_EQ(bp.stateAsString(rtt::ai::BallPossession::OURBALL), "OURBALL");
    EXPECT_EQ(bp.stateAsString(rtt::ai::BallPossession::THEIRBALL), "THEIRBALL");
    EXPECT_EQ(bp.stateAsString(rtt::ai::BallPossession::CONTENDEDBALL), "CONTENDEDBALL");
}


TEST(BallPossessionTest, team_far_or_close_to_ball) {
    rtt::ai::BallPossession bp;
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 8;
    field.field_length = 12;

    // teams close to ball
    auto worldmsg = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 0, true, field);
    rtt::ai::world::world->updateWorld(worldmsg.first);
    EXPECT_TRUE(bp.teamCloseToBall(world::world->getWorld(), true));
    EXPECT_FALSE(bp.teamFarFromBall(world::world->getWorld(), true));
    EXPECT_FALSE(bp.teamCloseToBall(world::world->getWorld(), false));

    worldmsg = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(0, 3, false, field);
    rtt::ai::world::world->updateWorld(worldmsg.first);
    EXPECT_FALSE(bp.teamCloseToBall(world::world->getWorld(), true));
    EXPECT_FALSE(bp.teamFarFromBall(world::world->getWorld(), false));
    EXPECT_TRUE(bp.teamCloseToBall(world::world->getWorld(), false));

    // put the bal out of the field
    // both teams should be far from the ball
    worldmsg.first.ball.pos.x = 100;
    worldmsg.first.ball.pos.y = 100;
    rtt::ai::world::world->updateWorld(worldmsg.first);

    // both are far from the ball
    EXPECT_TRUE(bp.teamFarFromBall(world::world->getWorld(), false));
    EXPECT_TRUE(bp.teamFarFromBall(world::world->getWorld(), true));

    // bot are not close
    EXPECT_FALSE(bp.teamCloseToBall(world::world->getWorld(), true));
    EXPECT_FALSE(bp.teamCloseToBall(world::world->getWorld(), false));
}

TEST(BallPossessionTest, it_properly_computes) {
    rtt::ai::BallPossession bp;
    roboteam_msgs::GeometryFieldSize field;
    field.field_width = 8;
    field.field_length = 12;


    auto worldmsg = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 0, true, field).first;
    worldmsg.time = 0.0;
    rtt::ai::world::world->updateWorld(worldmsg);

    worldmsg = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(3, 0, true, field).first;
    worldmsg.time = 1.0;
    rtt::ai::world::world->updateWorld(worldmsg);

    bp.update();
    EXPECT_EQ(bp.closeToUsTime, 1.0);
    EXPECT_EQ(bp.closeToThemTime, 0.0);
    EXPECT_EQ(bp.farFromUsTime, 0.0);
    EXPECT_EQ(bp.getPossession(), BallPossession::OURBALL);

    worldmsg = testhelpers::WorldHelper::getWorldMsgWhereRobotHasBall(0, 3, false, field).first;
    worldmsg.time = 2.0;
    rtt::ai::world::world->updateWorld(worldmsg);

    bp.update();
    EXPECT_EQ(bp.closeToUsTime, 0.0);
    EXPECT_EQ(bp.closeToThemTime, 2.0);
    EXPECT_EQ(bp.getPossession(), BallPossession::THEIRBALL);


}

}
}