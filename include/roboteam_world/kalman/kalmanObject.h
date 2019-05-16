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

        uint id;
        double observationTimeStamp;
        int invisibleCounter;
        bool exists;
        int comparisonCount;

        arma::fvec::fixed<stateIndex> X;
        arma::fvec::fixed<observationIndex> Z;
        arma::fmat::fixed<stateIndex, stateIndex> F;
        arma::fmat::fixed<observationIndex, stateIndex> H;
        arma::fmat::fixed<observationIndex, observationIndex> R;
        arma::fmat::fixed<stateIndex, stateIndex> I;
        arma::fmat::fixed<stateIndex, stateIndex> P;
        arma::fmat::fixed<stateIndex, stateIndex> Q;
        arma::fmat::fixed<stateIndex, observationIndex> K;

    public:

        kalmanObject();

        void kalmanUpdateK();

        void kalmanUpdateX();

        void kalmanUpdateZ(float x, float y, float z, double timeStamp);

        Position kalmanGetPos();

        Position kalmanGetVel();

        float getK();

        bool getExistance();

    };

}

#endif //ROBOTEAM_WORLD_KALMANOBJECT_H
