//
// Created by kjhertenberg on 13-5-19.
//

#include "kalman/kalmanObject.h"

kalmanObject::kalmanObject(){
    kalmanObject(INVALID_ROBOT_ID);
}

kalmanObject::kalmanObject(uint id){
    this->id = id;
    this->observationTimeStamp = 0.0;
    this->invisibleCounter = 0;
    this->exists = false;
    this->X.zeros();
    this->Z.zeros();
    this->F = {{1, TIMEDIFF, 0, 0, 0, 0 },
               {0, 1, 0, 0, 0, 0 },
               {0, 0, 1, TIMEDIFF, 0, 0 },
               {0, 0, 0, 1, 0, 0 },
               {0, 0, 0, 0, 1, TIMEDIFF },
               {0, 0, 0, 0, 0, 1} };
    this->H = {{1, 0, 0, 0, 0, 0 },
               {0, 0, 1, 0, 0, 0 },
               {0, 0, 0, 0, 1, 0 } };
    this->R = {{POS_VAR, 0, 0},
               {0, POS_VAR, 0},
               {0, 0, POS_VAR} };
    this->I.eyes();
    this->P = {{STATE_VAR, 0, 0, 0, 0, 0 },
               {0, STATE_VAR, 0, 0, 0, 0 },
               {0, 0, STATE_VAR, 0, 0, 0 },
               {0, 0, 0, STATE_VAR, 0, 0 },
               {0, 0, 0, 0, STATE_VAR, 0 },
               {0, 0, 0, 0, 0, STATE_VAR } };
    this->Q = {{TIMEDIFF*TIMEDIFF*RAND_VAR, TIMEDIFF*RAND_VAR, 0, 0, 0, 0 },
               {TIMEDIFF*RAND_VAR, RAND_VAR, 0, 0, 0, 0 },
               {0, 0, TIMEDIFF*TIMEDIFF*RAND_VAR, TIMEDIFF*RAND_VAR, 0, 0 },
               {0, 0, TIMEDIFF*RAND_VAR, RAND_VAR, 0, 0 },
               {0, 0, 0, 0, TIMEDIFF*TIMEDIFF*RAND_VAR, TIMEDIFF*RAND_VAR },
               {0, 0, 0, 0, TIMEDIFF*RAND_VAR, RAND_VAR } };
    this->K.zeros();
}

kalmanObject::~kalmanObject(){
    delete this->id;
    delete this->observationTimeStamp;
    delete this->invisibleCounter;
    delete this->exists;
    delete this->X;
    delete this->Z;
    delete this->F;
    delete this->H;
    delete this->R;
    delete this->I;
    delete this->P;
    delete this->Q;
    delete this->K;
}

void kalmanObject::kalmanUpdateK(){

    static int count = 0;

    if (count<100) {
        arma::fmat F_transpose = this->F.t();

        arma::fmat P_predict = (this->F * this->P * F_transpose) + this->Q;

        arma::fmat H_transpose = this->H.t();

        arma::fmat S = this->R + (this->H * P_predict * H_transpose);

        arma::fmat S_inverse = S.i();

        arma::fmat K_new = P_predict * H_transpose * S_inverse;

        arma::fmat IKH = this->I - K_new * this->H;

        arma::fmat P_new = IKH * P_predict * IKH.t() + K_new * this->R * K_new.t();

        int same = 0;
        for (arma::uword i = 0; i < STATE_INDEX; ++i) {
            for (arma::uword j = 0; j < OBSERVATION_INDEX; ++j) {
                if ((this->K(i, j) - K_new(i, j)) < 0.0000001 and (this->K(i, j) - K_new(i, j)) > 0.0000001)
                    same += 1;
            }
        }

        if (same == STATE_INDEX * OBSERVATION_INDEX) {
            count += 1;
        } else {
            count = 0;
        }

        this->K = K_new;
        this->P = P_new;
    }
}

void kalmanObject::kalmanUpdateX(){

    this->invisibleCounter += 1;
    if (this->invisibleCounter>200){
        //this->~kalmanObject();
        this->exists = false
    } else {

        arma::fmat X_predict = this->F * this->X;

        arma::fmat Y = this->Z - (this->H * X_predict);

        arma::fmat X_new = X_predict + (this->K * Y);

        this->X = X_new;

    }
}

void kalmanObject::kalmanUpdateZ(float x, float y, float z, double timeStamp){
    if (timestamp > this->observationTimeStamp) {
        this->Z(0, 0) = x;
        this->Z(1, 0) = y;
        this->Z(2, 0) = z;
        this->observationTimeStamp = timeStamp;
        this->invisibleCounter = 0;
        this->exists
    }
}

void kalmanObject::kalmanGetState(){

}
