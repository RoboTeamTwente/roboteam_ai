//
// Created by kjhertenberg on 16-5-19.
//

#include "kalman/KalmanBall.h"

namespace world {
KalmanBall::KalmanBall() {
    //initialise everything
    this->id = INVALID_ID; //ball has no id
    this->observationTimeStamp = - 1.0;
    this->invisibleCounter = 1000; //make sure the ball is invisible
    this->visibility = NOT_VISIBLE;
    this->exists = false;
    this->comparisonCount = 0;
    this->orientation = 0;
    this->omega = 0;
    this->cameraId = INVALID_ID;
    this->X.zeros();
    this->Z.zeros();
    this->F = {{1, TIMEDIFF, 0, 0},
               {0, 1, 0, 0},
               {0, 0, 1, TIMEDIFF},
               {0, 0, 0, 1}};
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
    this->Q = tempQ*tempQ_t*randVar_ball;
    this->K.zeros();
}

void KalmanBall::kalmanUpdateZ(proto::SSL_DetectionBall ball, double timeStamp, uint cameraID) {
    // if we have a ball already and the measurement is too far off we do not trust it.

    // convert mm to m
   float x = ball.x()/ 1000;
   float y = ball.y() /1000;
    float z = ball.z() /1000;

    if (visibility != NOT_VISIBLE) {
        //HAck
        float errorx = x - this->X(0);
        float errory = y - this->X(2);
        if (errorx*errorx + errory*errory
                >= 2) {
            return;
        }
    }
    else {
        // we found a new ball so we are resetting the state. We assume it's velocity is 0.
        std::cout<<"Jumping the ball"<<std::endl;
        this->pastObservation.clear();
        this->X.zeros();
        this->X(0) = x;
        this->X(2) = y;
    }
    rtt::Position average = calculatePos(rtt::Vector2(x,y), z, cameraID);
    this->cameraId = cameraID;
    this->Z(0) = average.x;
    this->Z(1) = average.y;

    // this is actually the height of the ball, but we are stupid
    this->omega = (average.rot - this->orientation)/(timeStamp - this->observationTimeStamp);
    this->orientation = average.rot;

    this->observationTimeStamp = timeStamp;
    this->invisibleCounter = 0;
    this->visibility = VISIBLE;
}

proto::WorldBall KalmanBall::as_ball_message() {
    //Same as the KalmanObject function but then for ball message
    proto::WorldBall msg;
    rtt::Position pos = kalmanGetPos();
    rtt::Position vel = kalmanGetVel();
    // since the balls z axis is being kept in the third place of the vector it is the 'rotation' here
    msg.set_area(1);
    msg.set_visible(isVisible());
    msg.mutable_pos()->set_x(pos.x);
    msg.mutable_pos()->set_y(pos.y);
    msg.set_z(pos.rot);
    msg.mutable_vel()->set_x(vel.x);
    msg.mutable_vel()->set_y(vel.y);
    msg.set_z_vel(vel.rot);

    return msg;
}

void KalmanBall::filterVel(rtt::Vector2 curVel) {

    double velocityDiff = (curVel - this->oldVel).length();
    double velForMaxFactor = 10.0;
    double maxFactor = 1.0;
    double factor = velocityDiff > velForMaxFactor ? maxFactor : velocityDiff/velForMaxFactor;
    rtt::Vector2 newVel = (this->oldVel*(1 - factor) + curVel*factor);
    this->oldVel.x = newVel.x;
    this->oldVel.y = newVel.y;
}

void KalmanBall::kalmanUpdateX() {
    // first we update the visibility and check if the ball has been seen the last time
    if (this->invisibleCounter>BALLEXTRAPOLATEDTIME&&visibility==EXTRAPOLATED){
        std::cout<<"Invisible ball! Not moving it anymore. "<<std::endl;
    }
    updateVisibility();

    // X_predict = FX_current
    // Y = Z - HX_predict
    // X_new = X_predict + Ky

    if (visibility!=NOT_VISIBLE){
        arma::fvec::fixed<STATEINDEX> X_predict = this->F*this->X;
        arma::fmat::fixed<OBSERVATIONINDEX, 1> Y;
        if (invisibleCounter<1){ // we only use the observation if we actually received one.
            Y= this->Z - (this->H*X_predict);
        }
        else{
            Y.zeros();
        }
        arma::fvec::fixed<STATEINDEX> X_new = X_predict + (this->K*Y);

        for (arma::uword i = 0; i < STATEINDEX; ++ i) {
            this->X(i) = X_new(i);
        }
    }
    this->invisibleCounter += 1; // we update the amount of loops which we did without seeing the ball (this is reset to 0 if the ball is seen again).
}
bool KalmanBall::isVisible() {
    return visibility==VISIBLE;
}
void KalmanBall::updateVisibility() {
    if (this->invisibleCounter>BALLDISAPPEARTIME){
        visibility=NOT_VISIBLE;
    }
    else if (this->invisibleCounter>BALLEXTRAPOLATEDTIME){
        visibility=EXTRAPOLATED;
    }
    else{
        visibility=VISIBLE;
    }
}
}