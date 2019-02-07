//
// Created by kjhertenberg on 18-12-18.
//

#ifndef ROBOTEAM_AI_CONTROLLER_H
#define ROBOTEAM_AI_CONTROLLER_H

#include "ControlUtils.h"
#include "../utilities/Constants.h"
#include <gtest/gtest_prod.h>

namespace rtt{
namespace ai {
namespace control {

class Controller {
    FRIEND_TEST(ControllerTest, it_calculates_proper_pid);

private:
    double kP = 0;
    double kI = 0;
    double kD = 0;
    double timeDiff = 1.0 / rtt::ai::Constants::getInt("tickRate");
    double initial_I = 0;
    double initial_I2 = 0; //only used in the case of 2 input variables
    double prev_error = 0;
    double prev_error2 = 0; //only used in the case of 2 input variables

public:
    Controller() = default;
    Controller(double P, double I, double D);
    Controller(double P, double I, double D, double timeDiff);
    Controller(double P, double I, double D, double timeDiff, double initial, double initial2, double prev, double prev2);

    void reset();

    void setP(double P);
    void setI(double I);
    void setD(double D);
    double getP();
    double getI();
    double getD();

    void setPID(double P, double I, double D);
    void setTimeDiff(double time);
    void setInitial(double initial);
    void setPrevErr(double prev);

    //overloaded functions
    void setP(double P, double time);
    void setI(double I, double time);
    void setD(double D, double time);
    void setPID(double P, double I, double D, double time);

    //controllers
    double controlP(double err);
    double controlI(double err);
    double controlD(double err);
    double controlR(double rate);
    Vector2 controlP(Vector2 err);
    Vector2 controlI(Vector2 err);
    Vector2 controlD(Vector2 err);
    Vector2 controlR(Vector2 rate);
    double controlPID(double err);

    //To fill in your own measured velocity
    double controlPIR(double err, double rate);

    //Same as above but for when we want to control 2 variables with the same k values
    //basically x and y
    Vector2 controlPID(Vector2 err);

    //To fill in your own measured velocity
    Vector2 controlPIR(Vector2 err, Vector2 rate);
};

} // control
} // ai
} // rtt

#endif //ROBOTEAM_AI_CONTROLLER_H
