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

        arma::fmat X;

        arma::fmat Z;

        arma::fmat F;

        arma::fmat H;

        arma::fmat R;

        arma::fmat I;

        arma::fmat P;

        arma::fmat Q;

        arma::fmat K;

    public:

        kalmanObject();

        kalmanObject(uint id);

        ~kalmanObject();

        void kalmanUpdateK();

        void kalmanUpdateX();

        void kalmanUpdateZ(float x, float y, float z, double timeStamp);

        Position kalmanGetState();
    };

}

#endif //ROBOTEAM_WORLD_KALMANOBJECT_H
