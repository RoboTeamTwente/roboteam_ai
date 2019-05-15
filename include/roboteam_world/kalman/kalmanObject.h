//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANOBJECT_H
#define ROBOTEAM_WORLD_KALMANOBJECT_H

#include "armadillo"
#include "roboteam_utils/Position.h"


#define STATE_INDEX 6
#define OBSERVATION_INDEX 3
#define TIMEDIFF 0.01
#define INVALID_ROBOT_ID 99999
#define COMPARISON_MARGIN 0.000001
#define COMPARISON_COUNT 100
#define TIME_TO_DISAPEAR 0.5*100
#define POS_VAR 0.5
#define STATE_VAR 1
#define RAND_VAR 1

namespace rtt {

    class kalmanObject {

    private:

        uint id;
        double observationTimeStamp;
        int invisibleCounter;
        bool exists;
        int comparisonCount;

        arma::fvec::fixed<STATE_INDEX> X;
        arma::fvec::fixed<OBSERVATION_INDEX> Z;
        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> F;
        arma::fmat::fixed<OBSERVATION_INDEX, STATE_INDEX> H;
        arma::fmat::fixed<OBSERVATION_INDEX, OBSERVATION_INDEX> R;
        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> I;
        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> P;
        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> Q;
        arma::fmat::fixed<STATE_INDEX, OBSERVATION_INDEX> K;

    public:

        kalmanObject();

        kalmanObject(uint id);

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
