//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANBALL_H
#define ROBOTEAM_WORLD_KALMANBALL_H

#include "kalmanObject.h"
#include "roboteam_msgs/DetectionBall.h"
#include "roboteam_msgs/WorldBall.h"
namespace rtt {

class kalmanBall : public kalmanObject {
private:
    Vector2 oldVel;
public:

    kalmanBall();

    //Same as the KalmanObject function but then for ball message
    roboteam_msgs::WorldBall as_ball_message();
    //Same as the KalmanObject function but then for ball frame
    void kalmanUpdateZ(roboteam_msgs::DetectionBall ball, double timestamp, uint cameraID);

    void filterVel(Vector2 curVel);
};

}

#endif //ROBOTEAM_WORLD_KALMANBALL_H
