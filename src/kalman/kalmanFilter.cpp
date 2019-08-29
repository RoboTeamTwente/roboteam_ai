//
// Created by kjhertenberg on 13-5-19.
//

#include <messages_robocup_ssl_detection.pb.h>
#include <roboteam_world/kalman/kalmanFilter.h>

#include "roboteam_world/kalman/kalmanFilter.h"

namespace rtt {

kalmanFilter::kalmanFilter() {
  std::lock_guard<std::mutex> lock(filterMutex);

  //initialise kalman objects
    lastFrameTime = - 1.0;
    for (uint i = 0; i < BOTCOUNT; ++ i) {
        ourBots[i] = kalmanUs(i);
        theirBots[i] = kalmanThem(i);
    }
    ball = kalmanBall();
}

void kalmanFilter::kalmanUpdate() {
  std::lock_guard<std::mutex> lock(filterMutex);

  //Updates the Kalman gain (K)
    //Updates the State (X)
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
void kalmanFilter::newFrame(const roboteam_proto::SSL_DetectionFrame &msg) {
  std::lock_guard<std::mutex> lock(filterMutex);

  double timeCapture = msg.t_capture();
    lastFrameTime = timeCapture;
    uint cameraID = msg.camera_id();
    for (const roboteam_proto::SSL_DetectionRobot& robot : msg.robots_yellow()) {
        ourBots[robot.robot_id()].kalmanUpdateZ(robot, timeCapture, cameraID);
    }
    for (const roboteam_proto::SSL_DetectionRobot& robot : msg.robots_blue()) {
        theirBots[robot.robot_id()].kalmanUpdateZ(robot, timeCapture, cameraID);
    }
    for (const roboteam_proto::SSL_DetectionBall& detBall : msg.balls()) {
        ball.kalmanUpdateZ(detBall, timeCapture, cameraID);
    }

}

//Creates a world message with the currently observed objects in it
roboteam_proto::World kalmanFilter::getWorld() {
  std::lock_guard<std::mutex> lock(filterMutex);

  roboteam_proto::World world;
    world.set_time(lastFrameTime);
    for (const auto& kalmanOurBot : ourBots){
        if (kalmanOurBot.getExistence()){
            world.mutable_yellow()->Add(kalmanOurBot.as_message());
        }
    }
    for (const auto& kalmanTheirBot : theirBots){
        if (kalmanTheirBot.getExistence()){
            world.mutable_blue()->Add(kalmanTheirBot.as_message());
        }
    }

    roboteam_proto::WorldBall worldBall = ball.as_ball_message();
    world.mutable_ball()->CopyFrom(worldBall);

  return world;
}
std::string kalmanFilter::getWorldMsg() {

  std::ostringstream stream;
  auto wrld = this->getWorld();
  std::lock_guard<std::mutex> lock(filterMutex);
  wrld.SerializeToOstream(&stream);

  return stream.str();
}

}