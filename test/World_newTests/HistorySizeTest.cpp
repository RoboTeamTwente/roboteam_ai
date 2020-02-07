//
// Created by emiel on 04-02-20.
//

#include <gtest/gtest.h>
#include <include/roboteam_ai/world_new/World.hpp>
#include <test/helpers/FieldHelper.h>
#include <test/helpers/WorldHelper.h>

TEST(World_newTest, HistorySizeTest){

    proto::GeometryFieldSize field = testhelpers::FieldHelper::generateField();
    proto::World world = testhelpers::WorldHelper::getWorldMsg(8, 8, false, field);

    // Get the worldInstance
    rtt::world_new::World* worldInstance = rtt::world_new::World::instance();

    // There should currently not be a world in the worldInstance
    EXPECT_FALSE(worldInstance->getWorld().has_value());
    // The history should be empty
    EXPECT_EQ(worldInstance->getHistorySize(), 0);

    // Put a world into worldInstance
    worldInstance->updateWorld(world);
    // These should now be a world in the worldInstance
    EXPECT_TRUE(worldInstance->getWorld().has_value());
    // The history of the world should still be empty
    EXPECT_EQ(worldInstance->getHistorySize(), 0);

    // Put another world into worldInstance
    worldInstance->updateWorld(world);
    // There should still be a world in the worldInstance
    EXPECT_TRUE(worldInstance->getWorld().has_value());
    // The history of the world should now contain a single world
    EXPECT_EQ(worldInstance->getHistorySize(), 1);

    // Fill up the worldInstance with twice the amount of maximum worlds
    for(int i = 0; i < 2 * rtt::world_new::World::HISTORY_SIZE; i++)
        worldInstance->updateWorld(world);

    // There should still be a world in the worldInstance
    EXPECT_TRUE(worldInstance->getWorld().has_value());
    // The history should contain exactly the amount of maximum worlds
    EXPECT_EQ(worldInstance->getHistorySize(), rtt::world_new::World::HISTORY_SIZE);

}

//TEST(World_newTest)