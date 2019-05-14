//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANOBJECT_H
#define ROBOTEAM_WORLD_KALMANOBJECT_H

#include <armadillo_bits/arma_forward.hpp>
#include "armadillo"
#include "roboteam_utils/Position.h"

#define STATE_INDEX 6
#define OBSERVATION_INDEX 3
#define TIMEDIFF 0.01
#define INVALID_ROBOT_ID 99999
#define POS_VAR 0.5
#define STATE_VAR 0.5
#define RAND_VAR 0.5

namespace rtt {

    class kalmanObject {

    private:

        uint id;
        double observationTimeStamp;
        int invisibleCounter;
        bool exists;

        arma::fmat X(STATE_INDEX, 1);

        arma::fmat Z(OBSERVATION_INDEX, 1);

        arma::fmat F(STATE_INDEX, STATE_INDEX);

        arma::fmat H(OBSERVATION_INDEX, STATE_INDEX);

        arma::fmat R(OBSERVATION_INDEX, OBSERVATION_INDEX);

        arma::fmat I(STATE_INDEX, STATE_INDEX);

        arma::fmat P(STATE_INDEX, STATE_INDEX);

        arma::fmat Q(STATE_INDEX, STATE_INDEX);

        arma::fmat K(STATE_INDEX, OBSERVATION_INDEX);

    public:

        kalmanObject();

        kalmanObject(uint id);

        ~kalmanObject();

        void kalmanUpdateK();

        void kalmanUpdateX();

        void kalmanUpdateZ(float x, float y, float z, double timeStamp);

        position kalmanGetState();
    };

}

#endif //ROBOTEAM_WORLD_KALMANOBJECT_H
