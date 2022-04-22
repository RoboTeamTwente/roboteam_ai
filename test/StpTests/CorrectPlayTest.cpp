//
// Created by alexander on 19-04-22.
//
#include <gtest/gtest.h>

#include "TestFixtures/TestFixture.h"
#include "stp/PlayDecider.hpp"

TEST_F(RTT_AI_Tests, KeeperKickBallTest) {
    using namespace rtt::ai::stp;

    // Generate a world and make sure the ball is in the defence area
    auto field = testhelpers::FieldHelper::generateField();
    auto protoWorld = testhelpers::WorldHelper::getWorldMsg(11, 11, true, field);
    auto ball = new proto::WorldBall;
    // Place ball in our defense area
    ball->mutable_pos()->set_x(-field.field_length() / 1000. / 2. + 0.5);
    ball->mutable_pos()->set_y(0);
    ball->mutable_vel()->set_x(0);
    ball->mutable_vel()->set_y(0);
    ball->set_visible(true);
    ball->set_area(99999);
    protoWorld.set_allocated_ball(ball);

    auto const& [_, world] = rtt::world::World::instance();
    world->updateWorld(protoWorld);
    world->updateField(field);
    auto bestPlay = PlayDecider::decideBestPlay(world, plays);
    // The ball is in the defense area and still, so we should do the Keeper Kick Ball play
    EXPECT_EQ(strcmp(bestPlay->getName(), "Keeper Kick Ball"), 0);
}

TEST_F(RTT_AI_Tests, scoringTest) {
    world = generateWorld();
    auto bestPlay = rtt::ai::stp::PlayDecider::decideBestPlay(world, plays);

    uint8_t bestScore = 0;
    for (auto& play : plays) {
        auto score = play->isValidPlayToStart() ? play->score(world->getField().value()) : 0;
        if (score > bestScore) bestScore = score;
    }

    EXPECT_EQ(bestScore, bestPlay->score(world->getField().value()));
}