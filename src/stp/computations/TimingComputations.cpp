//
// Created by alexander on 12-05-22.
//

#include "stp/computations/TimingComputations.h"
#include <cmath>

namespace rtt{
double TimingComputations::calculateBallTravelTime(const rtt::world::view::BallView& ball, double distance) {
    return std::pow(distance, 2) / 6.5;
}
}
