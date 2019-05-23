//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanBall.h"


namespace rtt {
    kalmanBall::kalmanBall() {
        //initialise everything
        this->id=INVALID_ID; //ball has no id
        this->observationTimeStamp = -1.0;
        this->invisibleCounter = 0;
        this->exists = false;
        this->comparisonCount = 0;
        this->orientation = 0;
        this->omega = 0;
        this->X.zeros();
        this->Z.zeros();
        this->F = {{1, TIMEDIFF, 0, 0       },
                   {0, 1,        0, 0       },
                   {0, 0,        1, TIMEDIFF},
                   {0, 0,        0, 1       }};
        this->H = {{1, 0, 0, 0},
                   {0, 0, 1, 0}};
        this->R = {{posVar_ball, 0},
                   {0, posVar_ball}};
        this->I.eye();
        this->P = {{stateVar_ball, 0, 0, 0},
                   {0, stateVar_ball, 0, 0},
                   {0, 0, stateVar_ball, 0},
                   {0, 0, 0, stateVar_ball}};
        arma::fmat::fixed<STATEINDEX, 2> tempQ = {{TIMEDIFF, 0},
                                                  {1, 0},
                                                  {0, TIMEDIFF},
                                                  {0, 1}};
        arma::fmat::fixed<2, STATEINDEX> tempQ_t = tempQ.t();
        this->Q = tempQ * tempQ_t * randVar_ball;
        this->K.zeros();
    }

    void kalmanBall::kalmanUpdateZ(roboteam_msgs::DetectionBall ball, double timeStamp) {
        //Same as the KalmanObject function but then for ball frame
        if (timeStamp > this->observationTimeStamp) {
            if (this->exists){
                //HAck
                float errorx = ball.pos.x-this->X(0);
                float errory = ball.pos.y-this->X(2);
                if (errorx*errorx+errory*errory >= 1){
                    return;
                }
            }
            this->Z(0) = ball.pos.x;
            this->Z(1) = ball.pos.y;
            this->omega = (ball.z - this->orientation)/(timeStamp-this->observationTimeStamp);
            this->orientation = ball.z;
            this->observationTimeStamp = timeStamp;
            this->invisibleCounter = 0;
            if (!this->exists){
                this->X.zeros();
                this->X(0) = ball.pos.x;
                this->X(2) = ball.pos.y;
            }
            this->exists = true;

        }
    }
    roboteam_msgs::WorldBall kalmanBall::as_ball_message() const{
        //Same as the KalmanObject function but then for ball message
        roboteam_msgs::WorldBall msg;
        Position pos =kalmanGetPos();
        Position vel =kalmanGetVel();
        // since the balls z axis is being kept in the third place of the vector it is the 'rotation' here
        msg.existence = 1;
        msg.visible = true;
        msg.pos.x = pos.x;
        msg.pos.y = pos.y;
        msg.z = pos.rot;
        msg.vel.x = vel.x;
        msg.vel.y = vel.y;
        msg.z_vel = vel.rot;
        return msg;
    }
}