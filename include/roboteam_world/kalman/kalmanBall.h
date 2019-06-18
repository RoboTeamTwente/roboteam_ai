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
        enum visState { VISIBLE, EXTRAPOLATED, NOT_VISIBLE };// three states; in first state we actively see the ball.
        // in EXTRAPOLATED we haven't seen the ball for a while but we do want the KALMAN filter to extrapolate it.
    private:
        Vector2 oldVel;
        visState visibility=NOT_VISIBLE;
    public:

        kalmanBall();

        //Same as the KalmanObject function but then for ball message
        roboteam_msgs::WorldBall as_ball_message();
        //Same as the KalmanObject function but then for ball frame
        bool isVisible();
        void kalmanUpdateZ(roboteam_msgs::DetectionBall ball, double timestamp, uint cameraID);
        void kalmanUpdateX() override;
        void updateVisibility();
        void filterVel(Vector2 curVel);

};

}

#endif //ROBOTEAM_WORLD_KALMANBALL_H
