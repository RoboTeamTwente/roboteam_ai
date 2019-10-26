//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANBALL_H
#define ROBOTEAM_WORLD_KALMANBALL_H

#include "KalmanObject.h"
#include "roboteam_proto/WorldBall.pb.h"
namespace world {

class KalmanBall : public KalmanObject {
 public:
  enum visState { VISIBLE, EXTRAPOLATED, NOT_VISIBLE };// three states; in first state we actively see the ball.
  // in EXTRAPOLATED we haven't seen the ball for a while but we do want the KALMAN filter to extrapolate it.
 private:
  rtt::Vector2 oldVel;
  visState visibility = NOT_VISIBLE;
 public:

  KalmanBall();

  //Same as the KalmanObject function but then for ball message
  roboteam_proto::WorldBall as_ball_message();
  //Same as the KalmanObject function but then for ball frame
  bool isVisible();
  void kalmanUpdateZ(roboteam_proto::SSL_DetectionBall ball, double timestamp, uint cameraID);
  void kalmanUpdateX() override;
  void updateVisibility();
  void filterVel(rtt::Vector2 curVel);

};

}

#endif //ROBOTEAM_WORLD_KALMANBALL_H
