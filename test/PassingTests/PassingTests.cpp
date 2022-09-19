#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <stp/computations/PassComputations.h>

#include "TestFixtures/TestFixture.h"
#include "helpers/FieldHelper.h"
#include "helpers/WorldHelper.h"
#include "stp/computations/PositionScoring.h"
#include "world/FieldComputations.h"

using namespace rtt::ai::stp;

TEST_F(RTT_AI_Tests, canFindPassTest) {
    PassInfo passInfo;
    int i = 0;
    // We should be able to find a valid pass in most situations- this checks that the requirements for passing aren't wrong causing no passes to be found
    do {
        world = generateWorld();
        passInfo = computations::PassComputations::calculatePass(gen::AttackingPass, world, world->getField().value());
        i++;
    } while (passInfo.passScore == 0 && i < 100);

    EXPECT_TRUE(passInfo.passScore != 0);
}

TEST_F(RTT_AI_Tests, idTests) {
    auto world = generateWorld();
    auto passInfo = computations::PassComputations::calculatePass(gen::AttackingPass, world, world->getField().value());
    EXPECT_TRUE(passInfo.passerId != passInfo.keeperId);
    EXPECT_TRUE(passInfo.passerId != passInfo.receiverId);
    EXPECT_TRUE(passInfo.receiverId != passInfo.keeperId);
    EXPECT_TRUE(passInfo.passerId != -1);
    EXPECT_TRUE(passInfo.keeperId != -1);
    EXPECT_TRUE((passInfo.receiverId != -1) || (passInfo.passScore == 0));

    auto us = world->getWorld()->getUs();
    auto keeperId = world->getWorld()->getRobotClosestToPoint(world->getField()->getOurGoalCenter(), us).value()->getId();
    erase_if(us, [keeperId](auto bot) { return bot->getId() == keeperId; });
    auto passerId = world->getWorld()->getRobotClosestToPoint(world->getWorld()->getBall()->get()->position, us).value()->getId();
    EXPECT_EQ(passInfo.passerId, passerId);
    EXPECT_TRUE(passInfo.keeperId == keeperId);
}

TEST_F(RTT_AI_Tests, validPassLocationTest) {
    auto world = generateWorld();
    auto passInfo = computations::PassComputations::calculatePass(gen::AttackingPass, world, world->getField().value());
    while (passInfo.passScore == 0) passInfo = computations::PassComputations::calculatePass(gen::AttackingPass, world, world->getField().value());

    EXPECT_TRUE(rtt::ai::FieldComputations::pointIsValidPosition(world->getField().value(), passInfo.passLocation));
    EXPECT_GT(PositionScoring::scorePosition(passInfo.passLocation, gen::LineOfSight, world->getField().value(), world).score, 0.0);
}

TEST_F(RTT_AI_Tests, keeperPassTest) {
    auto field = testhelpers::FieldHelper::generateField();
    auto protoWorld = testhelpers::WorldHelper::getWorldMsg(11, 11, true, field);
    auto ball = new proto::WorldBall;
    // Place ball in our goals centre
    ball->mutable_pos()->set_x(-6);
    ball->mutable_pos()->set_y(0);
    ball->mutable_vel()->set_x(0);
    ball->mutable_vel()->set_y(0);
    ball->set_visible(true);
    ball->set_area(99999);
    protoWorld.set_allocated_ball(ball);

    auto const& [_, world] = rtt::world::World::instance();
    world->updateWorld(protoWorld);
    world->updateField(field);

    ASSERT_TRUE(world->getWorld()->getBall()->get()->position == world->getField()->getOurGoalCenter());

    PassInfo passInfo;
    do {
        passInfo = computations::PassComputations::calculatePass(gen::AttackingPass, world, world->getField().value(), true);
    } while (passInfo.passScore == 0);

    auto keeper = world->getWorld()->getRobotClosestToPoint(world->getField()->getOurGoalCenter(), rtt::world::Team::us);
    ASSERT_TRUE(keeper.has_value());
    EXPECT_EQ(keeper->get()->getId(), passInfo.keeperId);
    EXPECT_EQ(passInfo.keeperId, passInfo.passerId);
    EXPECT_NE(passInfo.keeperId, passInfo.receiverId);
    EXPECT_TRUE(rtt::ai::FieldComputations::pointIsValidPosition(world->getField().value(), passInfo.passLocation));
    EXPECT_GT(PositionScoring::scorePosition(passInfo.passLocation, gen::LineOfSight, world->getField().value(), world).score, 0.0);
}