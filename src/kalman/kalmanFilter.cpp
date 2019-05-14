//
// Created by kjhertenberg on 13-5-19.
//

#include "kalman/kalmanFilter.h"
#include "kalman/KalmanObject.h"

void kalmanFilterInit(){
    //create 25 kalman objects (24 robots and 1 ball)
}

void kalmanUpdate(){
    //for all kalman objects
    // kalmanUpdateK();
    // kalmanUpdateX();
}

void newFrame(){
    //for each kalman object check if it's in the frame and update if the new data is more recent than the old data
    //kalmanUpdateZ();
}

void getStates(){
    //for each Kalman object that exists return it's state;
}