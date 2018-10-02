#include <gtest/gtest.h>
#include "roboteam_world/world/filtered_world.h"
#include "roboteam_world/world/world_dummy.h"
#include "roboteam_msgs/DetectionFrame.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/WorldBall.h"
#include <vector>

namespace rtt {

using namespace roboteam_msgs;
    
void dummy_frame(float ballx, float bally, float botx, float boty, float botw, DetectionFrame* frame) {
    DetectionRobot bot;
    bot.pos.x = botx;
    bot.pos.y = boty;
    bot.orientation = botw;
    frame->us.push_back(bot);
    DetectionBall ball;
    ball.pos.x = ballx;
    ball.pos.y = bally;
    frame->balls.push_back(ball);
}

void dummy_ball(float ballx, float bally, DetectionFrame* frame){
    DetectionBall ball;
    ball.pos.x=ballx;
    ball.pos.y=bally;
    frame->balls.push_back(ball);
}
void dummy_bot(float botx, float boty, float botw, DetectionFrame* frame){
    DetectionRobot bot;
    bot.pos.x = botx;
    bot.pos.y = boty;
    bot.orientation = botw;
    frame->us.push_back(bot);
}


//TODO: Expand testing to multiple robots for both colours
TEST(WorldTests, filtered) {
    int zero = 0;
    ros::init(zero, nullptr, "world_test");
    Predictor pred(1.0);
    FilteredWorld world(pred);

    ASSERT_FALSE(world.is_calculation_needed());
    auto * frame = new DetectionFrame();
    dummy_frame(1.0, 1.0, 2.0, 2.0, 0.0, frame);
    world.buffer_detection_frame(*frame);
    ASSERT_TRUE(world.is_calculation_needed());

    world.reset();
    world.detection_callback(*frame);
    ASSERT_TRUE(world.isFresh());
    ASSERT_FALSE(world.is_calculation_needed());
    world.consumeMsg();
    ASSERT_FALSE(world.isFresh());

    World msg = world.as_message();
    WorldRobot bot = msg.us[0];
    WorldBall ball = msg.ball;

    ASSERT_FLOAT_EQ(bot.pos.x, 2.0);
    ASSERT_FLOAT_EQ(bot.pos.y, 2.0);
    ASSERT_FLOAT_EQ(bot.angle, 0.0);

    ASSERT_FLOAT_EQ(ball.pos.x, 1.0);
    ASSERT_FLOAT_EQ(ball.pos.y, 1.0);

    DetectionRobot dr;

    frame->us.push_back(dr);
    frame->t_capture += 1;
    world.detection_callback(*frame);
    msg = world.as_message();
    bot = msg.us[0];

    ASSERT_FLOAT_EQ(-2.0, bot.vel.x);
    ASSERT_FLOAT_EQ(-2.0, bot.vel.y);
    ASSERT_FLOAT_EQ(0, fmod(bot.w, M_PI));

    }

    TEST(WorldTests, BallRobust){
        int zero = 0;
        ros::init(zero, nullptr, "world_test");
        Predictor pred(1.0);
        FilteredWorld world(pred);

        // Testing ball prediction.
        //First two frames to set the velocity and position of the ball
        auto * frame = new DetectionFrame();
        dummy_ball(0.0,0.0,frame);
        frame->t_capture=0;
        world.detection_callback(*frame);

        auto * frame2 = new DetectionFrame();
        dummy_ball(1.0,0.0,frame2);
        frame2->t_capture=1;
        world.detection_callback(*frame2);

        // Third frame has two noise balls in it. It should go for dummy_ball closest to expected position (at 2.0)
        auto * frame3 = new DetectionFrame();
        dummy_ball(2.0,0.0,frame3);
        dummy_ball(1.1,0.0,frame3);
        dummy_ball(-2.0,2.2,frame3);
        frame3->t_capture=2;
        world.detection_callback(*frame3);

        World msg=world.as_message();
        WorldBall ball=msg.ball;

        ASSERT_FLOAT_EQ(ball.vel.x,1.0);
        ASSERT_FLOAT_EQ(ball.vel.y,0.0);

        auto * frame4= new DetectionFrame();
        frame4->t_capture=3;
        world.detection_callback(*frame4);


        // 2 balls, one moving from 0.0 -> 1.0 and one moving from 2.0 -> 0.0
        // 2 robots, one moving from 0.0-> 2.0 and one moving from 1.0 -> 0.0


}
}
