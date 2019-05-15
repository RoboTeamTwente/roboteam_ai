//
// Created by kjhertenberg on 13-5-19.
//

#include <roboteam_world/kalman/kalmanObject.h>
namespace rtt {

    kalmanObject::kalmanObject() {
        kalmanObject(INVALID_ROBOT_ID);
    }

    kalmanObject::kalmanObject(uint id) {
        this->id = id;
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
        this->R = {{POS_VAR, 0, 0},
                   {0, POS_VAR, 0},
                   {0, 0, POS_VAR}};
        this->I.eye();
        this->P = {{STATE_VAR, 0, 0, 0, 0, 0},
                   {0, STATE_VAR, 0, 0, 0, 0},
                   {0, 0, STATE_VAR, 0, 0, 0},
                   {0, 0, 0, STATE_VAR, 0, 0},
                   {0, 0, 0, 0, STATE_VAR, 0},
                   {0, 0, 0, 0, 0, STATE_VAR}};
        this->Q = {{TIMEDIFF * TIMEDIFF * RAND_VAR, TIMEDIFF * RAND_VAR, 0, 0, 0, 0},
                   {TIMEDIFF * RAND_VAR, RAND_VAR, 0, 0, 0, 0},
                   {0, 0, TIMEDIFF * TIMEDIFF * RAND_VAR, TIMEDIFF * RAND_VAR, 0, 0},
                   {0, 0, TIMEDIFF * RAND_VAR, RAND_VAR, 0, 0},
                   {0, 0, 0, 0, TIMEDIFF * TIMEDIFF * RAND_VAR, TIMEDIFF * RAND_VAR},
                   {0, 0, 0, 0, TIMEDIFF * RAND_VAR, RAND_VAR}};
        this->K.zeros();
    }


    void kalmanObject::kalmanUpdateK() {

        if (this->comparisonCount < COMPARISON_COUNT) {

            /*
             * P = FPF^T+Q
             * S = R + HPH^T
             * K = PHS^-1
             * P = (I-KH)P(I-KH)^T+KRK^T
             */
            arma::fmat::fixed<STATE_INDEX, STATE_INDEX> F_transpose = this->F.t();
            arma::fmat::fixed<STATE_INDEX, STATE_INDEX> P_predict = (this->F * this->P * F_transpose) + this->Q;
            arma::fmat::fixed<STATE_INDEX, OBSERVATION_INDEX> H_transpose = this->H.t();
            arma::fmat::fixed<OBSERVATION_INDEX, OBSERVATION_INDEX> S = this->R + (this->H * P_predict * H_transpose);
            arma::fmat::fixed<OBSERVATION_INDEX, OBSERVATION_INDEX> S_inverse = S.i();
            arma::fmat::fixed<STATE_INDEX, OBSERVATION_INDEX> K_new = P_predict * H_transpose * S_inverse;
            arma::fmat::fixed<OBSERVATION_INDEX, STATE_INDEX> K_new_transpose = K_new.t();
            arma::fmat::fixed<STATE_INDEX, STATE_INDEX> IKH = this->I - K_new * this->H;
            arma::fmat::fixed<STATE_INDEX, STATE_INDEX> IKH_transpose = IKH.t();
            arma::fmat::fixed<STATE_INDEX, STATE_INDEX> P_new = IKH * P_predict * IKH_transpose + K_new * this->R * K_new_transpose;

            //See if the K has changed over the iteration
            float K_Diff_Max = (this->K - K_new).max();
            float K_Diff_Min = (this->K - K_new).min();
            int same = 0;
            if ((K_Diff_Max < COMPARISON_MARGIN) and (K_Diff_Min > -COMPARISON_MARGIN)){
                same += 1;
            }

            if (same == STATE_INDEX * OBSERVATION_INDEX) {
                this->comparisonCount += 1;
            } else {
                this->comparisonCount = 0;
            }

            for (arma::uword i = 0; i < STATE_INDEX; ++i) {
                for (arma::uword j = 0; j < OBSERVATION_INDEX; ++j) {
                    this->K(i, j) = K_new(i, j);
                }
                for (arma::uword k = 0; k < STATE_INDEX; ++k) {
                    this->P(i,k) = P_new(i,k);
                }
            }
        }
    }

    void kalmanObject::kalmanUpdateX() {

        this->invisibleCounter += 1;
        if (this->invisibleCounter > TIME_TO_DISAPEAR || !this->exists) {
            //this->~kalmanObject();
            this->exists = false;
        } else {

            arma::fvec::fixed<STATE_INDEX> X_predict = this->F * this->X;

            arma::fmat::fixed<OBSERVATION_INDEX, 1> Y = this->Z - (this->H * X_predict);

            arma::fvec::fixed<STATE_INDEX> X_new = X_predict + (this->K * Y);

            for (arma::uword i = 0; i < STATE_INDEX; ++i) {
                this->X(i) = X_new(i);
            }

        }
    }

    void kalmanObject::kalmanUpdateZ(float x, float y, float z, double timeStamp) {
        if (timeStamp > this->observationTimeStamp) {
            this->Z(0) = x;
            this->Z(1) = y;
            this->Z(2) = z;
            this->observationTimeStamp = timeStamp;
            this->invisibleCounter = 0;
            this->exists = true;
        }
    }


    Position kalmanObject::kalmanGetPos() {
        return {this->X(0), this->X(2), this->X(4)};
    }

    Position kalmanObject::kalmanGetVel() {
        return {this->X(1), this->X(3), this->X(5)};
    }

    float kalmanObject::getK(){
        return this->K(0, 0);
    }

    bool kalmanObject::getExistance(){
        return this->exists;
    }


}