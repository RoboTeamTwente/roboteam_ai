//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H
//TODO: add units to the below things, check with control/robothub.

namespace rtt {
namespace ai {
namespace constants {

//Mathematical constants
const double PI = 3.14159; // TODO: Why do we need this when we have M_PI from math.h? Conflicting usages? Global PI definition is very needed.

//skills
const double DEFAULT_KICK_POWER = 5.0; // max kick power = 100
const int MAX_KICK_CYCLES = 20;
const int MAX_GENEVA_CYCLES = 20;
const int DEFAULT_GENEVA_STATE = 0;

//Other/multiple usage
const int DEFAULT_ROBOT_ID = 1;
const double MAX_ANGULAR_VELOCITY = 4.0; // rad per second??

} // constants
} // ai
} // rtt

#endif //ROBOTEAM_AI_CONSTANTS_H
