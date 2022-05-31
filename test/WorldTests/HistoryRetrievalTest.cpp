//
// Created by emiel on 12-02-20.
//

#include <gtest/gtest.h>
#include <helpers/FieldHelper.h>
#include <helpers/WorldHelper.h>

#include <world/World.hpp>

#include "TestFixtures/TestFixture.h"

TEST_F(RTT_AI_Tests, HistoryRetrievalTest) {
    /** WORLD 1 - Yellow / Them - ID 1**/
    // Create robot 1
    proto::WorldRobot robot1;
    robot1.set_id(1);
    // Create a world and place the robot in it
    google::protobuf::RepeatedPtrField<proto::WorldRobot> robots1;
    robots1.Add()->CopyFrom(robot1);
    proto::World msg1;
    msg1.mutable_yellow()->CopyFrom(robots1);

    EXPECT_EQ(msg1.yellow().size(), 1);
    EXPECT_EQ(msg1.blue().size(), 0);
    EXPECT_EQ(msg1.yellow().at(0).id(), 1);

    /** WORLD 2 - Blue / Us - ID 2 **/
    // Create robot 2
    proto::WorldRobot robot2;
    robot2.set_id(2);
    // Create a world and place the robot in it
    google::protobuf::RepeatedPtrField<proto::WorldRobot> robots2;
    robots2.Add()->CopyFrom(robot2);
    proto::World msg2;
    msg2.mutable_blue()->CopyFrom(robots2);

    EXPECT_EQ(msg2.yellow().size(), 0);
    EXPECT_EQ(msg2.blue().size(), 1);
    EXPECT_EQ(msg2.blue().at(0).id(), 2);

    /** Test the WorldInstance / WorldManager **/
    auto const& [_, worldInstance] = rtt::world::World::instance();
    std::optional<rtt::world::view::WorldDataView> view;

    // Insert the world with a yellow robot, and check that the size of getUs() == 1 and getId() == 1
    worldInstance->updateWorld(msg1);
    view = worldInstance->getWorld();
    EXPECT_TRUE(view.has_value());
    EXPECT_EQ(view->getThem().size(), 0);  // Blue
    EXPECT_EQ(view->getUs().size(), 1);    // Yellow
    EXPECT_EQ(view->getUs().at(0)->getId(), 1);

    // Insert the next world, with a blue robot, which should be at getThem() and getId() == 2
    worldInstance->updateWorld(msg2);
    view = worldInstance->getWorld();
    EXPECT_TRUE(view.has_value());
    EXPECT_EQ(view->getThem().size(), 1);  // Blue
    EXPECT_EQ(view->getUs().size(), 0);    // Yellow
    EXPECT_EQ(view->getThem().at(0)->getId(), 2);

    // There should now be at least 1 world (yellow) in the history.
    // There might be more, due to world being a global static and being used by other tests
    EXPECT_GE(worldInstance->getHistorySize(), 1);

    // The first world (yellow) should now have been moved to history at ticksAgo = 1, so lets check that as well
    view = worldInstance->getHistoryWorld(1);
    EXPECT_TRUE(view.has_value());
    EXPECT_EQ(view->getThem().size(), 0);  // Blue
    EXPECT_EQ(view->getUs().size(), 1);    // Yellow
    EXPECT_EQ(view->getUs().at(0)->getId(), 1);

    // Lets keep pushing back worlds, and follow our worlds with our robot through the history
    // The Yellow world should be the oldest (them) with the Blue world following behind that (us)
    for (size_t index = 2; index <= rtt::world::World::HISTORY_SIZE; index++) {
        worldInstance->updateWorld(msg2);
        EXPECT_EQ(worldInstance->getHistoryWorld(index)->getUs().at(0)->getId(), 1);
        EXPECT_EQ(worldInstance->getHistoryWorld(index - 1)->getThem().at(0)->getId(), 2);
    }
}