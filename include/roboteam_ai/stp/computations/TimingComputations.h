//
// Created by alexander on 12-05-22.
//

#pragma once

#include "world/views/BallView.hpp"
namespace rtt {
class TimingComputations{
   public:
    /**
     * Calculates how long it takes the ball to travel the given distance
     * @param ball The ball
     * @param distance The distance for which you want to calculate the travel time
     * @return A rough estimation of how long it will take the ball to travel the given distance given the ball is not interacted with
     */
    static double calculateBallTravelTime(const world::view::BallView& ball, double distance);
};
}