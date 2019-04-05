// subscribe to the robotcommands channel
// run the kick skill
// check if the commands are on the robotcommands channel.

#include "ros/ros.h"
#include "../../src/skills/Kick.h"
#include "../../src/skills/Chip.h"
#include "../../src/world/World.h"
#include "../../src/utilities/RobotDealer.h"
#include "../../src/utilities/Constants.h"

// anonymous namespace needed to prevent ROS callback function name clashes
namespace {
std::vector<roboteam_msgs::RobotCommand> commands;

void robotCommandCallback(const roboteam_msgs::RobotCommandConstPtr &cmd) {
    commands.push_back(*cmd);
}

namespace w = rtt::ai::world;
namespace rd = rtt::ai::robotDealer;

TEST(KickTest, It_sends_proper_robotcommands) {
    ros::Rate rate(1);
    commands.clear(); // ensure the vector is empty.
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();
    bb->setInt("ROBOT_ID", 0);
    bb->setString("ROLE","test");
    rtt::ai::Kick kick("kicktest", bb);
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    robot.id=0;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);
    worldMsg.ball.existence = 99999;
    w::world->updateWorld(worldMsg);
    rd::RobotDealer::claimRobotForTactic(rd::RobotType::RANDOM,"KickTest","test");
    kick.initialize();
    EXPECT_EQ(kick.update(), bt::Leaf::Status::Running);

    // wait a little for the message to arrive and then spin
    rate.sleep();
    ros::spinOnce();

    std::vector<roboteam_msgs::RobotCommand> cmds = commands;
    EXPECT_EQ((signed int) commands.size(), 1);
    EXPECT_TRUE(commands.at(0).kicker);
    EXPECT_TRUE(commands.at(0).kicker_forced);
    EXPECT_EQ(commands.at(0).kicker_vel, rtt::ai::Constants::DEFAULT_KICK_POWER());

    bb->setDouble("kickVel", 2);
    rtt::ai::Kick kick2("test", bb);
    kick2.initialize();

    EXPECT_EQ(kick2.update(), bt::Leaf::Status::Running);

    // wait a little for the message to arrive and then spin
    rate.sleep();
    ros::spinOnce();

    EXPECT_EQ(commands.size(), (unsigned int) 2);
    EXPECT_EQ(commands.at(1).kicker_vel, 2);

    for (int i = 0; i < rtt::ai::Constants::MAX_KICK_CYCLES() - 1; i ++) {
        EXPECT_EQ(kick2.update(), bt::Leaf::Status::Running);
    }
    EXPECT_EQ(kick2.update(), bt::Leaf::Status::Failure);
    rd::RobotDealer::removeTactic("KickTest");
}

TEST(KickTest, It_chips) {
    ros::Rate rate(60);
    commands.clear(); // ensure the vector is empty.
    EXPECT_TRUE(commands.empty());
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<roboteam_msgs::RobotCommand>(rtt::TOPIC_COMMANDS, 0, &robotCommandCallback);

    auto bb = std::make_shared<bt::Blackboard>();
    bb->setInt("ROBOT_ID", 0);
    bb->setString("ROLE","test");
    rtt::ai::Chip chip("ChipTest", bb);
    roboteam_msgs::World worldMsg;
    roboteam_msgs::WorldRobot robot;
    robot.id=0;
    robot.pos.x=0;
    robot.pos.y=0;
    worldMsg.us.push_back(robot);
    worldMsg.ball.existence = 99999;
    w::world->updateWorld(worldMsg);
    rd::RobotDealer::claimRobotForTactic(rd::RobotType::RANDOM,"KickTest","test");

    chip.initialize();

    EXPECT_EQ(chip.update(), bt::Leaf::Status::Running);

    // wait a little for the message to arrive and then spin
    rate.sleep();
    ros::spinOnce();

    std::vector<roboteam_msgs::RobotCommand> cmds = commands;
    EXPECT_EQ((signed) commands.size(), 1);
    EXPECT_TRUE(commands.at(0).chipper);
    EXPECT_TRUE(commands.at(0).chipper_forced);
    EXPECT_EQ(commands.at(0).chipper_vel, rtt::ai::Constants::DEFAULT_KICK_POWER());
    rd::RobotDealer::removeTactic("KickTest");
}

}
