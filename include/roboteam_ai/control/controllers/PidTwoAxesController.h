//
// Created by ratoone on 25-09-19.
//

#ifndef RTT_PIDTWOAXESCONTROLLER_H
#define RTT_PIDTWOAXESCONTROLLER_H

#include <control/ControlUtils.h>
#include "BaseController.h"
#include "PidController.h"

/**
 * The PID class for 2 axes control (e.g. for 2D position or velocity control).
 * Makes use of two instances of PidController
 * @author Paul Vacariu
 * @SInCE 25.09.2019
 */
class PidTwoAxesController : BaseController<Vector2,Vector2>{
private:
    //the 2 PIDs used in the controller: one for the X axis, and one the Y axis
    PidController pidXAxis = PidController(0, 0, 0, 0);
    PidController pidYAxis = PidController(0, 0, 0, 0);

public:
    /**
     * Create a PID controller for two axes, with different parameters
     * @param pidXParams a tuple with 4 parameters for X dir PID - proportional, integrative, derivative and feedforward gain
     * @param pidYParams a tuple with 4 parameters for Y dir PID - proportional, integrative, derivative and feedforward gain
     */
    PidTwoAxesController(std::tuple<double, double, double, double> pidXParams, std::tuple<double, double, double, double> pidYParams);

    /**
     * Create a PID controller for two axes, with the same parameters for both of them
     * @param pidParams the PID parameters - proportional, integrative, derivative and feedforward
     */
    explicit PidTwoAxesController(std::tuple<double, double, double, double> pidParams);

    /**
     * Default constructor. All the values are initialized with zero.
     */
    PidTwoAxesController() = default;

    /**
     * Computes the PID output for the two axes
     * @param setPointInput the desired vector (e.g. position, velocity) values
     * @param actualInput the real vector values
     * @return the output is computed from the input parameters by using the 2 PIDs
     * @see PidController#computeOutput
     */
    Vector2 computeOutput(Vector2 setPointInput, Vector2 actualInput) override;

    /**
     * Sets the output limits for the two PID controllers
     * @param minimumX minimum value for the control output on X direction
     * @param maximumX maximum value for the control output on X direction
     * @param minimumY minimum value for the control output on Y direction
     * @param maximumY maximum value for the control output on Y direction
     */
    void setOutputLimits(double minimumX, double maximumX, double minimumY, double maximumY);

    /**
     * Sets the output ramp rate for the two PID controllers. This is the maximum
     * increment for which the control command can vary between ticks
     * @param rampRateX the max increment for X direction
     * @param rampRateY the max increment for Y direction
     */
    void setOutputRampRate(double rampRateX, double rampRateY);
};

#endif //RTT_PIDTWOAXESCONTROLLER_H
