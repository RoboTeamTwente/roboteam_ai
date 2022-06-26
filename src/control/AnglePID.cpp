//
// Created by rolf on 01-06-21.
//

#include "control/AnglePID.h"
#include <algorithm>

double rtt::AnglePID::getOutput(rtt::Angle target_angle, rtt::Angle current_angle) {
    // Calculate error; note that crossing the 'border' between two angles
    double dir = current_angle.rotateDirection(target_angle) ? 1.0 : -1.0;
    double error = dir * current_angle.shortestAngleDiff(target_angle);

    // Proportional term
    double Pout = P * error;

    // Integral term
    integral += error * dt;
    double Iout = I * integral;

    // Derivative term
    double derivative = (error - previous_error) / dt;
    double Dout = D * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    output = std::clamp(output, min, max);
    // Save error to previous error
    previous_error = error;

    return output;
}
