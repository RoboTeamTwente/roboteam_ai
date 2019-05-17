//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_world/kalman/kalmanFilter.h"

namespace rtt {

kalmanFilter::kalmanFilter() {
    lastFrameTime = - 1.0;
    for (uint i = 0; i < BOTCOUNT; ++ i) {
        ourBots[i] = kalmanUs(i);
        theirBots[i] = kalmanThem(i);
        ball = kalmanBall();
    }
}

void kalmanFilter::kalmanUpdate() {
    for (uint i = 0; i < BOTCOUNT; ++ i) {
        ourBots[i].kalmanUpdateK();
        ourBots[i].kalmanUpdateX();
        theirBots[i].kalmanUpdateK();
        theirBots[i].kalmanUpdateX();
    }
    ball.kalmanUpdateK();
    ball.kalmanUpdateX();
}

// if we get a new frame we update our observations
void kalmanFilter::newFrame(const roboteam_msgs::DetectionFrame &msg) {
    double timeCapture = msg.t_capture;
    lastFrameTime = timeCapture;
    for (const roboteam_msgs::DetectionRobot robot : msg.us) {
        ourBots[robot.robot_id].kalmanUpdateZ(robot, timeCapture);
    }
    for (const roboteam_msgs::DetectionRobot robot : msg.them) {
        theirBots[robot.robot_id].kalmanUpdateZ(robot, timeCapture);
    }
    for (const roboteam_msgs::DetectionBall detBall : msg.balls) {
        ball.kalmanUpdateZ(detBall, timeCapture);
    }
}

roboteam_msgs::World kalmanFilter::getWorld() {
    roboteam_msgs::World world;
    world.time = lastFrameTime;
    for (const auto& kalmanOurBot : ourBots){
        if (kalmanOurBot.getExistence()){
            world.us.push_back(kalmanOurBot.as_message());
        }
    }
    for (const auto& kalmanTheirBot : ourBots){
        if (kalmanTheirBot.getExistence()){
            world.us.push_back(kalmanTheirBot.as_message());
        }
    }
    if (ball.getExistence()){
        world.ball=ball.as_ball_message();
    }
    return world;
}

}