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
        this->cameraId = INVALID_ID;
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

    void kalmanBall::kalmanUpdateZ(roboteam_msgs::DetectionBall ball, double timeStamp, uint cameraID) {
        //Same as the KalmanObject function but then for ball frame
        if (this->exists){
            //HAck
            float errorx = ball.pos.x-this->X(0);
            float errory = ball.pos.y-this->X(2);
            if (errorx*errorx+errory*errory >= 2){//So the distance is the root of 2, 1.4 something? just being lazy, though apparently I have the time to write this comment
                return;
            }
        }
        if (!this->exists){
            this->pastObservation.clear();
            this->X.zeros();
            this->X(0) = ball.pos.x;
            this->X(2) = ball.pos.y;
        }
        Position average = calculatePos(ball.pos, ball.z, cameraID);
        this->cameraId = cameraID;
        this->Z(0) = average.x;
        this->Z(1) = average.y;
        this->omega = (average.rot - this->orientation)/(timeStamp-this->observationTimeStamp);
        this->orientation = average.rot;
        this->observationTimeStamp = timeStamp;
        this->invisibleCounter = 0;
        this->exists = true;
    }

    roboteam_msgs::WorldBall kalmanBall::as_ball_message(){
        //Same as the KalmanObject function but then for ball message
        roboteam_msgs::WorldBall msg;
        Position pos =kalmanGetPos();
        Position vel =kalmanGetVel();
        Vector2 curVel = {vel.x, vel.y};
        filterVel(curVel);
        // since the balls z axis is being kept in the third place of the vector it is the 'rotation' here
        msg.existence = 1;
        msg.visible = this->exists;
        if (this->exists){
            msg.pos.x = pos.x;
            msg.pos.y = pos.y;
        }
        else{
            msg.pos.x = lastSeenPos.x;
            msg.pos.y = lastSeenPos.y;
        }

        msg.z = pos.rot;
        msg.vel.x = this->oldVel.x;
        msg.vel.y = this->oldVel.y;
        msg.z_vel = vel.rot;
        return msg;
    }

    void kalmanBall::filterVel(Vector2 curVel){

        double velocityDiff = (curVel-this->oldVel).length();
        double velForMaxFactor = 10.0;
        double maxFactor = 1.0;
        double factor = velocityDiff > velForMaxFactor ? maxFactor : velocityDiff/velForMaxFactor;
        Vector2 newVel = (this->oldVel*(1 - factor) + curVel*factor);
        this->oldVel.x = newVel.x;
        this->oldVel.y = newVel.y;
    }

void kalmanBall::kalmanUpdateX() {

    this->invisibleCounter += 1;
    if (this->invisibleCounter > DISAPPEARTIME || !this->exists) {
        this->lastSeenPos=Vector2(kalmanGetPos().x,kalmanGetPos().y);
        this->exists = false;
    } else {
        // X_predict = FX_current
        // Y = Z - HX_predict
        // X_new = X_predict + Ky

        arma::fvec::fixed<STATEINDEX> X_predict = this->F * this->X;

        arma::fmat::fixed<OBSERVATIONINDEX, 1> Y = this->Z - (this->H * X_predict);

        arma::fvec::fixed<STATEINDEX> X_new = X_predict + (this->K * Y);

        for (arma::uword i = 0; i < STATEINDEX; ++i) {
            this->X(i) = X_new(i);
        }

    }
}
}