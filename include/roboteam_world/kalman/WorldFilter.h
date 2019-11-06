//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANFILTER_H
#define ROBOTEAM_WORLD_KALMANFILTER_H

#include "KalmanObject.h"
#include "KalmanRobot.h"
#include "KalmanBall.h"
#include "roboteam_utils/Position.h"
#include "roboteam_proto/WorldRobot.pb.h"
#include "roboteam_proto/World.pb.h"

namespace world {

//This class is a manager for the different Kalman object classes
class WorldFilter {
 private:
  double lastFrameTime;

 public:
  WorldFilter();
  void kalmanUpdate();
  void newFrame(const proto::SSL_DetectionFrame &msg);

  proto::World getWorld();
  KalmanRobot blueBots[BOTCOUNT];
  KalmanRobot yellowBots[BOTCOUNT];
  KalmanBall ball;

  std::mutex filterMutex;
};
}

#endif //ROBOTEAM_WORLD_KALMANFILTER_H
