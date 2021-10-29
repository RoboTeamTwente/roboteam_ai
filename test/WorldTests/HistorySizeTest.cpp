////
//// Created by emiel on 04-02-20.
////
//
//#include <gtest/gtest.h>
//#include <test/helpers/FieldHelper.h>
//#include <test/helpers/WorldHelper.h>
//#include <include/roboteam_ai/world/World.hpp>
//
//TEST(worldTest, HistorySizeTest) {
//    /* Note, this test only works if the history of worlds is still empty. Since its a static, make
//     * sure that this test is always the first to run for anything that uses or modifies this history! */
//    proto::World world;
//
//    // Get the worldInstance
//    rtt::world::World* worldInstance = rtt::world::World::instance();
//
//    // There should currently not be a world in the worldInstance
//    EXPECT_FALSE(worldInstance->getWorld().has_value());
//    // The history should be empty
//    EXPECT_EQ(worldInstance->getHistorySize(), 0);
//
//    // Put a world into worldInstance
//    worldInstance->updateWorld(world);
//    // These should now be a world in the worldInstance
//    EXPECT_TRUE(worldInstance->getWorld().has_value());
//    // The history of the world should still be empty
//    EXPECT_EQ(worldInstance->getHistorySize(), 0);
//
//    // Put another world into worldInstance
//    worldInstance->updateWorld(world);
//    // There should still be a world in the worldInstance
//    EXPECT_TRUE(worldInstance->getWorld().has_value());
//    // The history of the world should now contain a single world
//    EXPECT_EQ(worldInstance->getHistorySize(), 1);
//
//    // Fill up the worldInstance with twice the amount of maximum worlds
//    for (int i = 0; i < 2 * rtt::world::World::HISTORY_SIZE; i++) worldInstance->updateWorld(world);
//
//    // There should still be a world in the worldInstance
//    EXPECT_TRUE(worldInstance->getWorld().has_value());
//    // The history should contain exactly the amount of maximum worlds
//    EXPECT_EQ(worldInstance->getHistorySize(), rtt::world::World::HISTORY_SIZE);
//}