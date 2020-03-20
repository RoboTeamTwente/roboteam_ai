//
// Created by john on 3/2/20.
//
#define RUNNING_TEST

#include <gtest/gtest.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include <test/helpers/WorldHelper.h>

TEST(World_newTest, GenericWorldRemoval) {
    namespace w_n = rtt::world_new;
    proto::GeometryFieldSize size {};
    size.set_field_length(250);
    auto msg = testhelpers::WorldHelper::getWorldMsg(5, 7, true, size);
    auto second = msg;
    w_n::World::instance()->reset();
    w_n::World::instance()->updateWorld(msg);
    w_n::World::instance()->updateWorld(second);
    ASSERT_TRUE(w_n::World::instance()->getWorld().has_value());

    auto world = w_n::World::instance();
    world->reset();
    ASSERT_FALSE(world->getWorld().has_value());
}

TEST(World_newTest, HistorySizeTest) {
    namespace w_n = rtt::world_new;
    proto::GeometryFieldSize size {};
    size.set_field_length(250);
    auto msg = testhelpers::WorldHelper::getWorldMsg(5, 7, true, size);
    auto second = msg;
    w_n::World::instance()->reset();
    w_n::World::instance()->updateWorld(msg);
    w_n::World::instance()->updateWorld(second);
    ASSERT_EQ(w_n::World::instance()->getHistorySize(), 1);

    auto world = w_n::World::instance();
    world->reset();
    ASSERT_EQ(world->getHistorySize(), 0);
}

TEST(World_newTest, ResetWorldTest) {
    namespace w_n = rtt::world_new;
    proto::GeometryFieldSize size {};
    size.set_field_length(250);
    auto msg = testhelpers::WorldHelper::getWorldMsg(5, 7, true, size);
    auto second = msg;
    w_n::World::instance()->reset();
    w_n::World::instance()->updateWorld(msg);
    w_n::World::instance()->updateWorld(second);
    ASSERT_EQ(w_n::World::instance()->getWorld()->getUs().size(), 5);
    ASSERT_EQ(w_n::World::instance()->getWorld()->getThem().size(), 7);
    //ASSERT_TRUE(w_n::World::instance()->getWorld()->getBall().has_value());


    auto world = w_n::World::instance();
    world->reset();
    ASSERT_FALSE(world->getWorld().has_value());
}