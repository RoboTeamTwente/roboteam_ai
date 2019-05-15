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
#define STATE_VAR 1
#define RAND_VAR 1

namespace rtt {

    class kalmanObject {

    private:

        uint id;
        double observationTimeStamp;
        int invisibleCounter;
        bool exists;

        arma::fvec::fixed<STATE_INDEX> X;

        arma::fvec::fixed<OBSERVATION_INDEX> Z;

        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> F = {{1, TIMEDIFF, 0, 0,        0, 0},
                                                         {0, 1,        0, 0,        0, 0},
                                                         {0, 0,        1, TIMEDIFF, 0, 0},
                                                         {0, 0,        0, 1,        0, 0},
                                                         {0, 0,        0, 0,        1, TIMEDIFF},
                                                         {0, 0,        0, 0,        0, 1}};

        arma::fmat::fixed<OBSERVATION_INDEX, STATE_INDEX> H = {{1, 0, 0, 0, 0, 0},
                                                               {0, 0, 1, 0, 0, 0},
                                                               {0, 0, 0, 0, 1, 0}};

        arma::fmat::fixed<OBSERVATION_INDEX, OBSERVATION_INDEX> R = {{POS_VAR, 0, 0},
                                                                     {0, POS_VAR, 0},
                                                                     {0, 0, POS_VAR}};;

        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> I;

        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> P = {{STATE_VAR, 0, 0, 0, 0, 0},
                                                         {0, STATE_VAR, 0, 0, 0, 0},
                                                         {0, 0, STATE_VAR, 0, 0, 0},
                                                         {0, 0, 0, STATE_VAR, 0, 0},
                                                         {0, 0, 0, 0, STATE_VAR, 0},
                                                         {0, 0, 0, 0, 0, STATE_VAR}};

        arma::fmat::fixed<STATE_INDEX, STATE_INDEX> Q = {{TIMEDIFF * TIMEDIFF * RAND_VAR, TIMEDIFF * RAND_VAR, 0, 0, 0, 0},
                                                         {TIMEDIFF * RAND_VAR, RAND_VAR, 0, 0, 0, 0},
                                                         {0, 0, TIMEDIFF * TIMEDIFF * RAND_VAR, TIMEDIFF * RAND_VAR, 0, 0},
                                                         {0, 0, TIMEDIFF * RAND_VAR, RAND_VAR, 0, 0},
                                                         {0, 0, 0, 0, TIMEDIFF * TIMEDIFF * RAND_VAR, TIMEDIFF * RAND_VAR},
                                                         {0, 0, 0, 0, TIMEDIFF * RAND_VAR, RAND_VAR}};

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
