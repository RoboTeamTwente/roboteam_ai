//
// Created by kjhertenberg on 13-5-19.
//

#include <roboteam_world/kalman/kalmanObject.h>
namespace rtt {

    kalmanObject::kalmanObject() {
    }


    void kalmanObject::kalmanUpdateK() {

        if (this->comparisonCount < comparisonMaxAmount) {

            /*
             * P = FPF^T+Q
             * S = R + HPH^T
             * K = PHS^-1
             * P = (I-KH)P(I-KH)^T+KRK^T
             */
            arma::fmat::fixed<stateIndex, stateIndex> F_transpose = this->F.t();
            arma::fmat::fixed<stateIndex, stateIndex> P_predict = (this->F * this->P * F_transpose) + this->Q;
            arma::fmat::fixed<stateIndex, observationIndex> H_transpose = this->H.t();
            arma::fmat::fixed<observationIndex, observationIndex> S = this->R + (this->H * P_predict * H_transpose);
            arma::fmat::fixed<observationIndex, observationIndex> S_inverse = S.i();
            arma::fmat::fixed<stateIndex, observationIndex> K_new = P_predict * H_transpose * S_inverse;
            arma::fmat::fixed<observationIndex, stateIndex> K_new_transpose = K_new.t();
            arma::fmat::fixed<stateIndex, stateIndex> IKH = this->I - K_new * this->H;
            arma::fmat::fixed<stateIndex, stateIndex> IKH_transpose = IKH.t();
            arma::fmat::fixed<stateIndex, stateIndex> P_new = IKH * P_predict * IKH_transpose + K_new * this->R * K_new_transpose;

            //See if the K has changed over the iteration
            float K_Diff_Max = (this->K - K_new).max();
            float K_Diff_Min = (this->K - K_new).min();
            int same = 0;
            if ((K_Diff_Max < comparisonMargin) and (K_Diff_Min > -comparisonMargin)){
                same += 1;
            }

            if (same == stateIndex * observationIndex) {
                this->comparisonCount += 1;
            } else {
                this->comparisonCount = 0;
            }

            for (arma::uword i = 0; i < stateIndex; ++i) {
                for (arma::uword j = 0; j < observationIndex; ++j) {
                    this->K(i, j) = K_new(i, j);
                }
                for (arma::uword k = 0; k < stateIndex; ++k) {
                    this->P(i,k) = P_new(i,k);
                }
            }
        }
    }

    void kalmanObject::kalmanUpdateX() {

        this->invisibleCounter += 1;
        if (this->invisibleCounter > timeToDisapear || !this->exists) {
            //this->~kalmanObject();
            this->exists = false;
        } else {

            arma::fvec::fixed<stateIndex> X_predict = this->F * this->X;

            arma::fmat::fixed<observationIndex, 1> Y = this->Z - (this->H * X_predict);

            arma::fvec::fixed<stateIndex> X_new = X_predict + (this->K * Y);

            for (arma::uword i = 0; i < stateIndex; ++i) {
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