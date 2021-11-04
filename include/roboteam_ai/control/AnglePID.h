//
// Created by rolf on 01-06-21.
//

#ifndef RTT_ANGLEPID_H
#define RTT_ANGLEPID_H

#include "roboteam_utils/Angle.h"
#include "roboteam_utils/pid.h"
namespace rtt {
class AnglePID {
   private:
    double P;
    double I;
    double D;
    double min;
    double max;
    double previous_error;
    double integral;
    double dt;

   public:
    AnglePID(double P, double I, double D, double max_ang_vel, double dt) : P{P}, I{I}, D{D}, min{-max_ang_vel}, max{max_ang_vel}, previous_error{0.0}, integral{0.0}, dt{dt} {}

    double getOutput(Angle target_angle, Angle current_angle);
};
}  // namespace rtt

#endif  // RTT_ANGLEPID_H
