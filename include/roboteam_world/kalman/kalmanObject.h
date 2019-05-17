//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANOBJECT_H
#define ROBOTEAM_WORLD_KALMANOBJECT_H

#include "armadillo"
#include "roboteam_utils/Position.h"
#include "constantsK.h"
#include "roboteam_msgs/DetectionRobot.h"

namespace rtt {

class kalmanObject {

    protected:
        int id;
        double observationTimeStamp;
        int invisibleCounter;
        bool exists;
        int comparisonCount;
        double orientation;
        double omega;

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

        void kalmanUpdateZ(roboteam_msgs::DetectionRobot robot,double timeStamp);

        Position kalmanGetPos() const;

        Position kalmanGetVel() const;

        float getK();

        bool getExistence() const;

        virtual roboteam_msgs::WorldRobot as_message() const;

};

}

#endif //ROBOTEAM_WORLD_KALMANOBJECT_H
