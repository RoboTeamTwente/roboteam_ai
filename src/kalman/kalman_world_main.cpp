
#include "ros_handler.h"


int main(int argc, char** argv) {
    double memory_time = 0.1;
    rtt::Predictor predictor(memory_time);
    rtt::FilteredWorld world(predictor);

    rtt::RosHandler handler;
    handler.init(&world);
    handler.setKalman(true);
    std::cout << "---- Kalman world ready. ----" << std::endl;
    handler.kalmanLoop();
    return 0;
}