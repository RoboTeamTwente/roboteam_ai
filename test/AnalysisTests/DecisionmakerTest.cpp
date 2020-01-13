//
// Created by mrlukasbos on 16-4-19.
//

#include <gtest/gtest.h>
#include <analysis/DecisionMaker.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include "world_old/World.h"
#include <utilities/RobotDealer.h>

namespace rtt {
namespace ai {
namespace analysis {

TEST(DecisionMakerTest, all_setups_have_right_amounts_of_robots) {


    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;

    // for all possible amounts of robots
    for (int amountOfRobots = 1; amountOfRobots < 9; amountOfRobots++) {

        // set up a field
        roboteam_msgs::World worldmsg = testhelpers::WorldHelper::getWorldMsg(amountOfRobots, 0, false, field);
        world::world->updateWorld(worldmsg);

        // iterate over the ballPossession enum
        for (int fooInt = THEY_HAVE_BALL; fooInt != WE_HAVE_BALL; fooInt++ ) {
            auto possession = static_cast<BallPossession>(fooInt);

            // there is a keeper but it is not visible (the id is set to -1, and robot -1 is never seen, thus mimicking an invisible keeper)
            robotDealer::RobotDealer::setKeeperID(-1);
            DecisionMaker maker;
            PlayStyle style = maker.getRecommendedPlayStyle(possession);
            int total = style.amountOfAttackers + style.amountOfMidfielders + style.amountOfDefenders;
            EXPECT_EQ(total, std::max(0, amountOfRobots));

            // there is a keeper
            robotDealer::RobotDealer::setKeeperID(world::world->getUs().at(0)->id);

            style = maker.getRecommendedPlayStyle(possession);
            total = style.amountOfAttackers + style.amountOfMidfielders + style.amountOfDefenders;
            EXPECT_EQ(total, std::max(0, amountOfRobots-1));
        }
    }
}

}
}
}