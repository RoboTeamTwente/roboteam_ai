//
// Created by kjhertenberg on 16-5-19.
//

#include "roboteam_world/kalman/kalmanBall.h"


namespace rtt {
    kalmanBall::kalmanBall() {
        this->id=INVALID_ID;
        this->observationTimeStamp = -1.0;
        this->invisibleCounter = 0;
        this->exists = false;
        this->comparisonCount = 0;
        this->X.zeros();
        this->Z.zeros();
        this->F = {{1, TIMEDIFF, 0, 0,        0, 0},
                   {0, 1,        0, 0,        0, 0},
                   {0, 0,        1, TIMEDIFF, 0, 0},
                   {0, 0,        0, 1,        0, 0},
                   {0, 0,        0, 0,        1, TIMEDIFF},
                   {0, 0,        0, 0,        0, 1}};
        this->H = {{1, 0, 0, 0, 0, 0},
                   {0, 0, 1, 0, 0, 0},
                   {0, 0, 0, 0, 1, 0}};
        this->R = {{posVar_ball, 0, 0},
                   {0, posVar_ball, 0},
                   {0, 0, posVar_ball}};
        this->I.eye();
        this->P = {{stateVar_ball, 0, 0, 0, 0, 0},
                   {0, stateVar_ball, 0, 0, 0, 0},
                   {0, 0, stateVar_ball, 0, 0, 0},
                   {0, 0, 0, stateVar_ball, 0, 0},
                   {0, 0, 0, 0, stateVar_ball, 0},
                   {0, 0, 0, 0, 0, stateVar_ball}};
        this->Q = {{TIMEDIFF * TIMEDIFF * randVar_ball, TIMEDIFF * randVar_ball, 0, 0, 0, 0},
                   {TIMEDIFF * randVar_ball, randVar_ball, 0, 0, 0, 0},
                   {0, 0, TIMEDIFF * TIMEDIFF * randVar_ball, TIMEDIFF * randVar_ball, 0, 0},
                   {0, 0, TIMEDIFF * randVar_ball, randVar_ball, 0, 0},
                   {0, 0, 0, 0, TIMEDIFF * TIMEDIFF * randVar_ball, TIMEDIFF * randVar_ball},
                   {0, 0, 0, 0, TIMEDIFF * randVar_ball, randVar_ball}};
        this->K.zeros();
    }
    void kalmanBall::kalmanUpdateZ(roboteam_msgs::DetectionBall ball, double timestamp) {
        if (timestamp > this->observationTimeStamp) {
            this->Z(0) = ball.pos.x;
            this->Z(1) = ball.pos.y;
            this->Z(2) = ball.z;
            this->observationTimeStamp = timestamp;
            this->invisibleCounter = 0;
            this->exists = true;
        }
    }
    roboteam_msgs::WorldBall kalmanBall::as_ball_message() const{
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