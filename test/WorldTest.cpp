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
        Predictor pred(10);
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
        dummy_bot(0.0,0.0,0.0,frame4);
        frame4->t_capture=3;
        world.detection_callback(*frame4);

        auto * frame5 = new DetectionFrame();
        dummy_bot(1.0,0.0,0.0, frame5);
        frame5->t_capture=4;
        world.detection_callback(*frame5);

        auto * frame6 = new DetectionFrame();

        // It simply picks the last one that was pushed if Robot ID's are the same. Practically never happens during a Match
        dummy_bot(1.9,0.0,0.0,frame6);
        dummy_bot(2.5,0.0,0.0,frame6);
        frame6->t_capture=5;
        world.detection_callback(*frame6);

        msg=world.as_message();
        WorldRobot bot=msg.us[0];

}
TEST(WorldTests,BallRobustMerge){
    // Tests whether the merging of frames happens correctly

    // Two camera's alternating. Ball moves from id 0 to id 1
        int zero = 0;
        ros::init(zero, nullptr, "world_test");
        Predictor pred(10);
        FilteredWorld world(pred);

        auto * frame = new DetectionFrame();
        dummy_ball(0.0,0.0,frame);
        frame->t_capture=0;
        frame->camera_id=0;
        world.detection_callback(*frame);

        auto * frame2 = new DetectionFrame();
        frame2->t_capture=1;
        frame2->camera_id=1;
        world.detection_callback(*frame2);

        auto * frame3 = new DetectionFrame();
        dummy_ball(2.0,0.0,frame3);
        frame3->t_capture=2;
        frame3->camera_id=0;
        world.detection_callback(*frame3);

        World msg=world.as_message();
        WorldBall ball=msg.ball;
        ASSERT_FLOAT_EQ(ball.vel.x,1.0);
        ASSERT_FLOAT_EQ(ball.vel.y,0.0);

        auto * frame4 = new DetectionFrame();
        frame4->t_capture=3;
        frame4->camera_id=1;
        world.detection_callback(*frame4);

        auto * frame5 = new DetectionFrame();
        dummy_ball(4.0,0.0,frame5);
        frame5->t_capture=4;
        frame5->camera_id=0;
        world.detection_callback(*frame5);

        auto * frame6 = new DetectionFrame();
        dummy_ball(5.15,0.0,frame6);
        frame6->t_capture=5;
        frame6->camera_id=1;
        world.detection_callback(*frame6);

        auto * frame7 = new DetectionFrame();
        dummy_ball(10,0.0,frame7);
        frame7->t_capture=6;
        frame7->camera_id=0;
        world.detection_callback(*frame7);

        msg=world.as_message();
        ball=msg.ball;
        ASSERT_FLOAT_EQ(ball.vel.x,1.05);
}
TEST(WorldTests,RobotMerge){
        // Similar to above, but for Robots merging
        int zero = 0;
        ros::init(zero, nullptr, "world_test");
        Predictor pred(10);
        FilteredWorld world(pred);

        auto * frame = new DetectionFrame();
        dummy_bot(0.0,0.0,0.0,frame);
        frame->t_capture=0;
        frame->camera_id=0;
        world.detection_callback(*frame);

        auto * frame2 = new DetectionFrame();
        frame2->t_capture=1;
        frame2->camera_id=1;
        world.detection_callback(*frame2);

        auto * frame3 = new DetectionFrame();
        dummy_bot(2.0,0.0,0.0,frame3);
        frame3->t_capture=2;
        frame3->camera_id=0;
        world.detection_callback(*frame3);

        World msg=world.as_message();
        WorldRobot bot=msg.us[0];
        ASSERT_FLOAT_EQ(bot.vel.x,1.0);
        ASSERT_FLOAT_EQ(bot.vel.y,0.0);

        auto * frame4 = new DetectionFrame();
        frame4->t_capture=3;
        frame4->camera_id=1;
        world.detection_callback(*frame4);

        auto * frame5 = new DetectionFrame();
        dummy_bot(4.0,0.0,0.0,frame5);
        frame5->t_capture=4;
        frame5->camera_id=0;
        world.detection_callback(*frame5);

        auto * frame6 = new DetectionFrame();
        dummy_bot(5.15,0.0,0.0,frame6);
        frame6->t_capture=5;
        frame6->camera_id=1;
        world.detection_callback(*frame6);

        auto * frame7 = new DetectionFrame();
        dummy_bot(10,0.0,0.0,frame7);
        frame7->t_capture=6;
        frame7->camera_id=0;
        world.detection_callback(*frame7);

        msg=world.as_message();
        bot=msg.us[0];
        //ASSERT_FLOAT_EQ(bot.pos.x,(5.15+10)/2); // Old. Simply merge together the Robot position and make that the new position.
        ASSERT_FLOAT_EQ(bot.pos.x,10); //New, works for now because we have no fail save checking and take the latest frame.
}
}


