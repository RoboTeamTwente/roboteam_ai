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
    public:
        roboteam_msgs::WorldBall as_ball_message() const;
        void kalmanUpdateZ(roboteam_msgs::DetectionBall ball, double timestamp);
        kalmanBall();
};

}

#endif //ROBOTEAM_WORLD_KALMANBALL_H
