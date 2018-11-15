//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_CONSTANTS_H
#define ROBOTEAM_AI_CONSTANTS_H
//TODO: add units to the below things, check with control/robothub.

namespace rtt{
namespace ai{

class Constants {
        // Const ints and const doubles are handled differently in C++ because ints are integral type but doubles are not (insert rant about C++ magic here).
        // As far as I know there is no pretty solution where we can keep everything in the .h file. If you have one, please use it. Or, we go back to macros.
    public:
        //Mathematical constants
        const static double PI(){return 3.14159;} // TODO: Why do we need this when we have M_PI from math.h? Conflicting usages? Global PI definition is very needed.
        //Kick
        const static double DEFAULT_KICK_POWER(){return 5.0;} // max kick power = 100
        static const int MAX_KICK_CYCLES=20;
        //Geneva
        static const int MAX_GENEVA_CYCLES=20;
        static const int DEFAULT_GENEVA_STATE=0;
        //Other/multiple usage
        static const int DEFAULT_ROBOT_ID=1;
        const static double MAX_ANGULAR_VELOCITY(){ return 4.0;} // rad per second??
};
}
}

#endif //ROBOTEAM_AI_CONSTANTS_H
