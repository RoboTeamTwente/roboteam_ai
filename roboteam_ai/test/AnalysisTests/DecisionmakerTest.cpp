//
// Created by mrlukasbos on 16-4-19.
//

#include <gtest/gtest.h>
#include <roboteam_ai/src/analysis/DecisionMaker.h>
#include <roboteam_ai/test/helpers/WorldHelper.h>
#include <roboteam_ai/src/world/World.h>
#include <roboteam_ai/src/utilities/RobotDealer.h>

namespace rtt {
namespace ai {
namespace analysis {

TEST(DecisionMakerTest, all_setups_have_right_amounts_of_robots) {

    robotDealer::RobotDealer::setUseSeparateKeeper(false);

    roboteam_msgs::GeometryFieldSize field;
    field.field_length = 12;
    field.field_width = 9;

    // for all possible amounts of robots
    for (int amountOfRobots = 0; amountOfRobots < 9; amountOfRobots++) {

        // set up a field
        roboteam_msgs::World worldmsg = testhelpers::WorldHelper::getWorldMsg(amountOfRobots, 0, false, field);
        world::world->updateWorld(worldmsg);

        // iterate over the ballplacement enum
        for (int fooInt = THEY_HAVE_BALL; fooInt != WE_HAVE_BALL; fooInt++ ) {
            auto possession = static_cast<BallPossession>(fooInt);
            DecisionMaker maker;
            PlayStyle style = maker.getRecommendedPlayStyle(possession);

            // the total amount of robots from the playstyle should always equal
            int total = style.amountOfAttackers + style.amountOfMidfielders + style.amountOfDefenders;
            EXPECT_EQ(total, amountOfRobots);
        }
    }
}

}
}
}