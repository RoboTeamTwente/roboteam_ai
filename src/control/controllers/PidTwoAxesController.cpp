//
// Created by ratoone on 25-09-19.
//

#include <control/controllers/PidTwoAxesController.h>

PidTwoAxesController::PidTwoAxesController(std::tuple<double, double, double, double> pidXParams, std::tuple<double, double, double, double> pidYParams){
    pidXAxis = PidController(pidXParams);
    pidYAxis = PidController(pidYParams);
}

Vector2 PidTwoAxesController::computeOutput(Vector2 setPointInput, Vector2 actualInput) {
    Vector2 outputCommand;
    outputCommand.x = pidXAxis.computeOutput(setPointInput.x, actualInput.x);
    outputCommand.y = pidYAxis.computeOutput(setPointInput.y, actualInput.y);
    return outputCommand;
}

PidTwoAxesController::PidTwoAxesController(std::tuple<double, double, double, double> pidParams)
        : PidTwoAxesController(pidParams, pidParams){}
