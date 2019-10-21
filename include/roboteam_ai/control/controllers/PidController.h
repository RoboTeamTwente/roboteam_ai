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
 * and outputs a float controller action. For more info check https://en.wikipedia.org/wiki/PID_controller
 */
class PidController : BaseController<double,double>{
public:
    /** Create a new PID object.
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain.	Becomes large if setpoint cannot reach target quickly.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
     */
    PidController(double p, double i, double d);
    /** Create a new PID object.
     * @param p Proportional gain. Large if large difference between setpoint and target.
     * @param i Integral gain.	Becomes large if setpoint cannot reach target quickly.
     * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
     * @param f Feedforward gain. The value that will be multiplied with the input and added to the output
     */
    PidController(double p, double i, double d, double f);
    /** Create a new PID object
     * @oaram pid tuple with the three parameters of the PID
     */
    explicit PidController(std::tuple<double, double, double> pid);
    /** Create a new PID object
     * @oaram pid tuple with the three parameters of the PID, plus feedforward gain
     */
    explicit PidController(std::tuple<double, double, double, double> pidf);

    void setPID(double, double, double);
    void setPID(std::tuple<double, double, double>);
    void setPID(std::tuple<double, double, double>, double);

    void setPID(double, double, double, double);
    /**Set the maximum output value contributed by the I component of the system
     * this->can be used to prevent large windup issues and make tuning simpler
     * @param maximum. Units are the same as the expected output value
     */
    void setMaxIOutput(double maximum);
    /**Specify a maximum output. If a single parameter is specified, the minimum is
     * set to (-maximum).
     * @param output
     */
    void setOutputLimits(double output);
    /**
     * Specify a maximum output.
     * @param minimum possible output value
     * @param maximum possible output value
     */
    void setOutputLimits(double minimum, double maximum);
    /** Set the operating direction of the PID controller
     * @param reversed Set true to reverse PID output
     */
    void setDirection(bool reversed);
    /**
     * Resets the controller. this->erases the I term buildup, and removes D gain on the next loop.
     */
    void reset();
    /**Set the maximum rate the output can increase per cycle.
     * @param rate
     */
    void setOutputRampRate(double);
    /** Set a limit on how far the setpoint can be from the current position
     * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range.
     * <br>this->limits the reactivity of P term, and restricts impact of large D term
     * during large setpoint adjustments. Increases lag and I term if range is too small.
     * @param range
     */
    void setSetpointRange(double);
    /**Set a filter on the output to reduce sharp oscillations. <br>
     * 0.1 is likely a sane starting value. Larger values P and D oscillations, but force larger I values.
     * Uses an exponential rolling sum filter, according to a simple <br>
     * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
     * @param output valid between [0..1), meaning [current output only.. historical output only)
     */
    void setOutputFilter(double);
    /** Calculate the PID value needed to hit the target setpoint.
    * Automatically re-calculates the output at each call.
    * @param actual The monitored value
    * @param setpoint The target value
    * @return calculated output value for driving the actual to the target
    */
    double computeOutput(double setPoint, double actualValue) override;

private:
    /**
     * Forces a value into a specific range
     * @param value input value
     * @param min maximum returned value
     * @param max minimum value in range
     * @return Value if it's within provided range, min or max otherwise
     */
    double clamp(double, double, double);
    /**
     * Test if the value is within the min and max, inclusive
     * @param value to test
     * @param min Minimum value of range
     * @param max Maximum value of range
     * @return
     */
    bool bounded(double, double, double);
    /**
     * To operate correctly, all PID parameters require the same sign,
     * with that sign depending on the {@literal}reversed value
     */
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
