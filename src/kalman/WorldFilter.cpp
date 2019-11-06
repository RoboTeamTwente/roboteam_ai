//
// Created by kjhertenberg on 13-5-19.
//

#include "roboteam_proto/messages_robocup_ssl_detection.pb.h"
#include <kalman/WorldFilter.h>

namespace world {

WorldFilter::WorldFilter() {
  std::lock_guard<std::mutex> lock(filterMutex);

  //initialise kalman objects
    lastFrameTime = - 1.0;
    for (uint i = 0; i < BOTCOUNT; ++ i) {
        yellowBots[i] = KalmanRobot(i);
        blueBots[i] = KalmanRobot(i);
    }
    ball = KalmanBall();
}

void WorldFilter::kalmanUpdate() {
  std::lock_guard<std::mutex> lock(filterMutex);

  //Updates the Kalman gain (K)
    //Updates the State (X)
    for (uint i = 0; i < BOTCOUNT; ++ i) {
        yellowBots[i].kalmanUpdateK();
        yellowBots[i].kalmanUpdateX();
        blueBots[i].kalmanUpdateK();
        blueBots[i].kalmanUpdateX();
    }
    ball.kalmanUpdateK();
    ball.kalmanUpdateX();


}

// if we get a new frame we update our observations
void WorldFilter::newFrame(const proto::SSL_DetectionFrame &msg) {
  std::lock_guard<std::mutex> lock(filterMutex);

  double timeCapture = msg.t_capture();
    lastFrameTime = timeCapture;
    uint cameraID = msg.camera_id();
    for (const proto::SSL_DetectionRobot& robot : msg.robots_yellow()) {
        yellowBots[robot.robot_id()].kalmanUpdateZ(robot, timeCapture, cameraID);
    }
    for (const proto::SSL_DetectionRobot& robot : msg.robots_blue()) {
        blueBots[robot.robot_id()].kalmanUpdateZ(robot, timeCapture, cameraID);
    }
    for (const proto::SSL_DetectionBall& detBall : msg.balls()) {
        ball.kalmanUpdateZ(detBall, timeCapture, cameraID);
    }

}

//Creates a world message with the currently observed objects in it
proto::World WorldFilter::getWorld() {
  std::lock_guard<std::mutex> lock(filterMutex);

  proto::World world;
    world.set_time(lastFrameTime);
    for (const auto& kalmanYellowBot : yellowBots){
        if (kalmanYellowBot.getExistence()){
            world.mutable_yellow()->Add(kalmanYellowBot.as_message());
        }
    }
    for (const auto& kalmanBlueBot : blueBots){
        if (kalmanBlueBot.getExistence()){
            world.mutable_blue()->Add(kalmanBlueBot.as_message());
        }
    }

    proto::WorldBall worldBall = ball.as_ball_message();
    world.mutable_ball()->CopyFrom(worldBall);

  return world;
}

}