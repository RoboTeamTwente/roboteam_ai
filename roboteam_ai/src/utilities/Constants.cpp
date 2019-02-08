//
// Created by mrlukasbos on 8-2-19.
//

#include "Constants.h"

namespace rtt {
namespace ai {

// static initializers
bool Constants::isInitialized = false;
bool Constants::robotOutputTargetGrSim = true;

 void Constants::init() {
    ros::NodeHandle nh;
    std::string robotOutputTarget;
    nh.getParam("robot_output_target", robotOutputTarget);
    robotOutputTargetGrSim = robotOutputTarget != "serial"; // only use serial if it is explicitly defined
    std::cout << "robot_output_target = " << (robotOutputTargetGrSim ? "GRSIM" : "SERIAL") << std::endl;
    isInitialized = true;
}

 bool Constants::GRSIM() {
    if (!isInitialized) std::cerr << "Ros::init() was not called yet, but you use a value that depends on a ROS parameter. \n this may result in unexepected behaviour" std::endl;
    return robotOutputTargetGrSim;
}

}
}