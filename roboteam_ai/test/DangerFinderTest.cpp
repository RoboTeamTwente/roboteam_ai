//
// Created by mrlukasbos on 5-10-18.
//

#include <gtest/gtest.h>
#include "../src/dangerfinder/DangerFinder.h"
#include <bitset>

namespace df = rtt::ai::dangerfinder;

// fieldMsg is the same everywhere
roboteam_msgs::GeometryFieldSize fieldMsg;
df::DangerData danger;

void setFieldtoWorld() {
    // set the field parameters
    fieldMsg.field_length = 1000;
    fieldMsg.field_width = 500;
    fieldMsg.goal_width = 200;
    fieldMsg.goal_depth = 20;

    // set the field to the world
    rtt::ai::Field::set_field(fieldMsg);
}

// return a robot at a given location
roboteam_msgs::WorldRobot getRobot(int x, int y, int id = 0) {
    roboteam_msgs::WorldRobot robot;
    robot.pos = rtt::Vector2(x, y);
    robot.id = (unsigned int) id;
    return robot;
}

// return a ball at a given location
roboteam_msgs::WorldBall getBall(int x, int y) {
    roboteam_msgs::WorldBall ball;
    ball.pos = rtt::Vector2(x, y);
    return ball;
}

// short function to get results from dangerfinder
df::DangerData calculateDangerForWorld(roboteam_msgs::World world) {
    rtt::ai::World::set_world(world);
    return df::DangerFinder::instance().calculateDataNow();
}

// Sets some robots on the field and checks the danger flags
TEST(DangerFinderTest, it_has_correct_danger_flags) {
    roboteam_msgs::World worldMsg;
    setFieldtoWorld();

    ASSERT_FALSE(df::DangerFinder::instance().hasCalculated());
    // there is nothing between enemy robot and the ball
    worldMsg.ball = getBall(0, 0);
    worldMsg.them.push_back(getRobot(0, 400));
    danger = calculateDangerForWorld(worldMsg);

    ASSERT_TRUE(df::DangerFinder::instance().hasCalculated());
    ASSERT_EQ(danger.flags.size(), (unsigned int) 1);
    ASSERT_EQ(std::bitset<8>(danger.flags.at(0)),
            std::bitset<8>(0b00000001)); // the robotflag ISFREE should be triggered

    // now put one of our robots between them and the ball
    worldMsg.us.push_back(getRobot(0, 200));
    danger = calculateDangerForWorld(worldMsg);

    ASSERT_EQ(danger.flags.size(), (unsigned int) 1);
    ASSERT_EQ(std::bitset<8>(danger.flags.at(0)), std::bitset<8>(0b00000000));

    // the robot should now also be closing in on our goal
    worldMsg.them.push_back(getRobot(0, 500));
    danger = calculateDangerForWorld(worldMsg);
    ASSERT_EQ((signed) danger.flags.size(), 1);
    ASSERT_EQ(std::bitset<8>(danger.flags.at(0)), std::bitset<8>(0b00000010));

    df::DangerFinder::instance().stop();
}

// Creates an amount of robots in the world and looks if it contains useful dangerdata
TEST(DangerFinderTest, it_logs_dangerdata) {
    setFieldtoWorld();
    roboteam_msgs::World worldMsg;

    int amountOfRobots = 11;
    // set some robots (at exactly the same location for testing purposes)
    for (int i = 0; i < amountOfRobots; i ++) {
        worldMsg.us.push_back(getRobot(100, 200, i));
        worldMsg.them.push_back(getRobot(100, 200, i));
    }
    danger = calculateDangerForWorld(worldMsg);

    // All robots of the enemy should have dangerscores
    ASSERT_EQ((signed) danger.dangerList.size(), amountOfRobots);
    ASSERT_EQ((signed) danger.scores.size(), amountOfRobots);
    ASSERT_EQ((signed) danger.flags.size(), amountOfRobots);
    // there is no other data set for the robots so their danger scores should be equal
    for (int i = 1; i < (signed) danger.scores.size(); i ++) {
        ASSERT_EQ(danger.scores.at(i - 1), danger.scores.at(i));
    }

    // add a ball to the field
    worldMsg.ball = getBall(0, 0);
    danger = calculateDangerForWorld(worldMsg);

    // there is still no different data set for the robots so their danger scores should be equal
    for (int i = 1; i < (signed) danger.scores.size(); i ++) {
        ASSERT_EQ(danger.scores.at(i - 1), danger.scores.at(i));
    }

    // move one robot forward with the ball
    // and point him towards the goal
    worldMsg.them.at(0).pos = rtt::Vector2(200, 0);
    worldMsg.them.at(0).angle = 28*M_PI;
    worldMsg.ball.pos = rtt::Vector2(250, 0);

    danger = calculateDangerForWorld(worldMsg);

    // his dangerscore should be sky high (because of canshoot)
    // the canshoot module adds a score of 999, so it should be at least that value.
    ASSERT_GE(danger.scores[danger.getByDangerRank(0)->id], 999);

    df::DangerFinder::instance().stop();
}

// creates a lot of random positions in the field and checks if dangerscores stay within bounds
TEST(DangerFinderTest, it_stays_within_limits) {
    setFieldtoWorld();
    for (int i = 0; i < 1000; i ++) {
        roboteam_msgs::World tempWorldMsg;
        // set robots at random locations
        int amountOfRobots = 11;
        for (int i = 0; i < amountOfRobots; i ++) {
            // set robots at random locations in the field
            int halfFieldLength = (int) fieldMsg.field_length/2;
            int halfFieldWidth = (int) fieldMsg.field_width/2;
            int randomX = rand()%(halfFieldLength*2 + 1) + halfFieldLength;
            int randomY = rand()%(halfFieldWidth*2 + 1) + halfFieldWidth;

            tempWorldMsg.us.push_back(getRobot(randomX, randomY, i));
            tempWorldMsg.them.push_back(getRobot(randomX, randomY, i));
        }
        danger = calculateDangerForWorld(tempWorldMsg);

        // values can never be smaller than 0 and never greater than 1300
        for (unsigned int j = 0; j < danger.scores.size(); j ++) {
            ASSERT_GE(danger.scores.at(j), 0);
            ASSERT_LE(danger.scores.at(j), 1300);
        }
    }
}

// checks if most_recent_data equals the data that is calculated immediately
TEST(DangerFinderTest, most_recent_data_is_correct) {
    setFieldtoWorld();
    roboteam_msgs::World worldMsg;

    int amountOfRobots = 11;
    // set some robots (at exactly the same location for testing purposes)
    for (int i = 0; i < amountOfRobots; i ++) {
        worldMsg.us.push_back(getRobot(100, 200, i));
        worldMsg.them.push_back(getRobot(100, 200, i));
    }
    danger = calculateDangerForWorld(worldMsg);

    // calculatedNow data should equal mostRecentData
    df::DangerData danger2 = df::DangerFinder::instance().getMostRecentData();
    EXPECT_EQ(danger.scores, danger2.scores);
    EXPECT_EQ(danger.flags, danger2.flags);
    df::DangerFinder::instance().stop();

}

// checks the + operator on partialresult
// the scores are added together and the flags are OR'd
TEST(DangerFinderTest, partial_results_can_be_added_together) {
    PartialResult pr1;
    PartialResult pr2;
    PartialResult pr3;

    pr1.score = 3;
    pr1.flags = 0b11001100;
    pr2.score = 4;
    pr2.flags = 0b00110011;

    pr3 = pr1 + pr2;

    ASSERT_EQ(pr3.score, 7);
    ASSERT_EQ(pr3.flags, 0b11111111);

    pr1.score = - 3;
    pr1.flags = 0b00000000;
    pr2.score = 4;
    pr2.flags = 0b00110011;

    pr3 = pr1 + pr2;

    ASSERT_EQ(pr3.score, 1);
    ASSERT_EQ(pr3.flags, 0b00110011);
    df::DangerFinder::instance().stop();
}
