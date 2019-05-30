//
// Created by kjhertenberg on 13-5-19.
//

#ifndef ROBOTEAM_WORLD_KALMANOBJECT_H
#define ROBOTEAM_WORLD_KALMANOBJECT_H

#include "armadillo"
#include "roboteam_utils/Position.h"
#include "constantsK.h"
#include "roboteam_msgs/DetectionRobot.h"
#include "roboteam_utils/Vector2.h"

namespace rtt {

    //class for objects who's data goes trough the kalman filter
    //Based on: https://en.wikipedia.org/wiki/Kalman_filter
class kalmanObject {

    protected:
        int id; //Id of the object if applicable
        double observationTimeStamp; //Time of last observed data used to make sure old data doesn't replace new data
        int invisibleCounter; //count the ticks between observations, after a certain time the object doesn't exist anymore
        bool exists; //if true we consider the object to be existing
        int comparisonCount; //time the iteration of P and K where they are the same
        float orientation; //currently the filter only filters X and Y, du to the coordinate system
        double omega; //""
        uint cameraId;
        std::vector<Position> pastObservation;

        // The key matrices (fixed size to prevent side effects)
        arma::fvec::fixed<STATEINDEX> X; //Constains the state of the robot
        arma::fvec::fixed<OBSERVATIONINDEX> Z; //contains the lates observations
        arma::fmat::fixed<STATEINDEX, STATEINDEX> F; //dynamics how current state should change tot the next state
        arma::fmat::fixed<OBSERVATIONINDEX, STATEINDEX> H; //How the state should project to the observation
        arma::fmat::fixed<OBSERVATIONINDEX, OBSERVATIONINDEX> R; //The covariance of the observation noise
        arma::fmat::fixed<STATEINDEX, STATEINDEX> I; //Identity matrix
        arma::fmat::fixed<STATEINDEX, STATEINDEX> P; //A measure of the estimated accuracy of the state estimate
        arma::fmat::fixed<STATEINDEX, STATEINDEX> Q; // The covariance of the process noise (random forces)
        arma::fmat::fixed<STATEINDEX, OBSERVATIONINDEX> K; //Kalman gain, found based on the variances inputted.

    public:

        //updates the K till it doesn't change anymore (with wrong variance it can osscilate or grow exponetially)
        void kalmanUpdateK();

        //If the object exists, updates the state
        void kalmanUpdateX();

        //if the data is more recent than the current data, import the new observation data
        void kalmanUpdateZ(roboteam_msgs::DetectionRobot robot,double timeStamp, int cameraID);

        //Get X,Y and Orientation
        Position kalmanGetPos() const;

        //Get X_vel, Y_vel and omega
        Position kalmanGetVel() const;

        //Get K for debug
        float getK();

        //Does the object exist
        bool getExistence() const;

        //Create a message, by default it's a robot message (the ball object overrides this)
        virtual roboteam_msgs::WorldRobot as_message() const;

        double limitRotation(double rotation) const;

        Position calculatePos(Vector2 pos, float rot, int camID);

};

}

#endif //ROBOTEAM_WORLD_KALMANOBJECT_H
