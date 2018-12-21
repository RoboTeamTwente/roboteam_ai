//
// Created by rolf on 11/12/18.
//

#ifndef ROBOTEAM_AI_PID_H
#define ROBOTEAM_AI_PID_H
#include "ControlUtils.h"
namespace rtt{
    namespace ai {
        namespace control {
class PID {
    public:
        void setParams(double pGainPos, double iGainPos, double dGainPos, double pGainRot, double iGainRot,
                double dGainRot);
        void initialize(double timeDif);
        Vector2 posControl(Vector2 myPos, Vector2 targetPos);
        Vector2 posControl(Vector2 myPos, Vector2 targetPos, Vector2 myVel);

    private:
        double pGainPos;
        double iGainPos;
        double dGainPos;
        double pGainRot;
        double iGainRot;
        double dGainRot;

        Vector2 prevPosErr;
        double timeDif;
        Vector2 I;

};

        } // control
    } // ai
} // rtt

#endif //ROBOTEAM_AI_PID_H
