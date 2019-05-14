//
// Created by kjhertenberg on 14-5-19.
//

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


    TEST(KalmanTest, bot) {



    }
}