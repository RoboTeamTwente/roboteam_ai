//
// Created by kjhertenberg on 16-5-19.
//

#ifndef ROBOTEAM_WORLD_CONSTANTSK_H
#define ROBOTEAM_WORLD_CONSTANTSK_H

namespace  rtt {

    const int stateIndex = 6;
    const int observationIndex = 3;
    const float timeDiff = 0.01;
    const float comparisonMargin = 0.000001;
    const int comparisonMaxAmount = 100;
    const float timeToDisapear = 0.5*100;
    const float posVar = 0.5;
    const float stateVar = 1;
    const float randVar = 1;
    const float posVar_us = 0.5;
    const float stateVar_us = 1;
    const float randVar_us = 1;
    const float posVar_them = 0.5;
    const float stateVar_them = 1;
    const float randVar_them = 1;
    const float posVar_ball = 0.5;
    const float stateVar_ball = 1;
    const float randVar_ball = 1;

};

#endif //ROBOTEAM_WORLD_CONSTANTSK_H
