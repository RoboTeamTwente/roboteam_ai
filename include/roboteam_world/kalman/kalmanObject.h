//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANOBJECT_H
#define ROBOTEAM_WORLD_KALMANOBJECT_H

#include "armadillo"
#include "roboteam_utils/Position.h"
#include "constantsK.h"

namespace rtt {

class kalmanObject {

    protected:
        int id;
        double observationTimeStamp;
        int invisibleCounter;
        bool exists;
        int comparisonCount;

        // see https://en.wikipedia.org/wiki/Kalman_filter for explanation
        arma::fvec::fixed<STATEINDEX> X;
        arma::fvec::fixed<OBSERVATIONINDEX> Z;
        arma::fmat::fixed<STATEINDEX, STATEINDEX> F;
        arma::fmat::fixed<OBSERVATIONINDEX, STATEINDEX> H;
        arma::fmat::fixed<OBSERVATIONINDEX, OBSERVATIONINDEX> R;
        arma::fmat::fixed<STATEINDEX, STATEINDEX> I;
        arma::fmat::fixed<STATEINDEX, STATEINDEX> P;
        arma::fmat::fixed<STATEINDEX, STATEINDEX> Q;
        arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> K;

    public:

        void kalmanUpdateK();

        void kalmanUpdateX();

        void kalmanUpdateZ(float x, float y, float z, double timeStamp);

        Position kalmanGetPos();

        Position kalmanGetVel();

        float getK();

        bool getExistence();

};

}

#endif //ROBOTEAM_WORLD_KALMANOBJECT_H
