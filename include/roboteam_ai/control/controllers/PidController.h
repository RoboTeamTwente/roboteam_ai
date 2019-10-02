//
// Created by mrlukasbos on 29-3-19.
//

#ifndef ROBOTEAM_AI_PID_H
#define ROBOTEAM_AI_PID_H

#include <tuple>
#include "BaseController.h"

/**
 * The basic implementation of a PIDF (proportional - integrative - derivative
 * - feedback) controller. The actual controller accepts a float reference
 * and outputs a float controller action
 */
class PidController : BaseController<double,double>{
public:
    PidController(double, double, double);
    PidController(double, double, double, double);
    explicit PidController(std::tuple<double, double, double>);
    explicit PidController(std::tuple<double, double, double, double>);

    void setPID(double, double, double);
    void setPID(std::tuple<double, double, double>);
    void setPID(std::tuple<double, double, double>, double);

    void setPID(double, double, double, double);
    void setMaxIOutput(double);
    void setOutputLimits(double);
    void setOutputLimits(double,double);
    void setDirection(bool);
    void reset();
    void setOutputRampRate(double);
    void setSetpointRange(double);
    void setOutputFilter(double);
    double computeOutput(double setPoint, double actualValue) override;

private:
    double clamp(double, double, double);
    bool bounded(double, double, double);
    void checkSigns();
    double P;
    double I;
    double D;
    double F;

    double maxIOutput;
    double maxError;
    double errorSum;

    double maxOutput;
    double minOutput;

    double setpoint;

    double lastActual;

    bool firstRun;
    bool reversed;

    double outputRampRate;
    double lastOutput;

    double outputFilter;

    double setpointRange;
};

#endif //ROBOTEAM_AI_PID_H
