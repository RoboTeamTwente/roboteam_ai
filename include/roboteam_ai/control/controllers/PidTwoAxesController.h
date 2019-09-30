//
// Created by ratoone on 25-09-19.
//

#ifndef RTT_PIDTWOAXESCONTROLLER_H
#define RTT_PIDTWOAXESCONTROLLER_H

#include <include/roboteam_ai/control/ControlUtils.h>
#include "BaseController.h"
#include "PidController.h"

class PidTwoAxesController : BaseController<Vector2,Vector2>{
public:
    PidController pidXAxis = PidController(0, 0, 0, 0);
    PidController pidYAxis = PidController(0, 0, 0, 0);

    PidTwoAxesController(std::tuple<double, double, double, double> pidXParams, std::tuple<double, double, double, double> pidYParams);

    explicit PidTwoAxesController(std::tuple<double, double, double, double> pidParams);

    Vector2 computeOutput(Vector2 setPointInput, Vector2 actualInput) override;

    PidTwoAxesController() = default;
};

#endif //RTT_PIDTWOAXESCONTROLLER_H
