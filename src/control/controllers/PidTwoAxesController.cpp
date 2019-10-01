//
// Created by ratoone on 25-09-19.
//

#include <control/controllers/PidTwoAxesController.h>

/**
 * Create a PID controller for two axes, with different parameters
 * @param pidXParams a tuple with 4 parameters for X dir PID - proportional, integrative, derivative and feedforward gain
 * @param pidYParams a tuple with 4 parameters for Y dir PID - proportional, integrative, derivative and feedforward gain
 */
PidTwoAxesController::PidTwoAxesController(std::tuple<double, double, double, double> pidXParams, std::tuple<double, double, double, double> pidYParams){
    pidXAxis = PidController(pidXParams);
    pidYAxis = PidController(pidYParams);
}

/**
 * Computes the PID output for the two axes
 * @param setPointInput the setpoint expressed as a Vector2
 * @param actualInput the real value expressed as a Vector2
 * @return the output Vector2 is computed from the input parameters by using the PID
 * @see PidController#computeOutput
 */
Vector2 PidTwoAxesController::computeOutput(Vector2 setPointInput, Vector2 actualInput) {
    Vector2 outputCommand;
    outputCommand.x = pidXAxis.computeOutput(setPointInput.x, actualInput.x);
    outputCommand.y = pidYAxis.computeOutput(setPointInput.y, actualInput.y);
    return outputCommand;
}

/**
 * Create a PID controller for two axes, with the same parameters for both of them
 * @param pidParams the PID parameters - proportional, integrative, derivative and feedforward
 */
PidTwoAxesController::PidTwoAxesController(std::tuple<double, double, double, double> pidParams)
        : PidTwoAxesController(pidParams, pidParams){}
