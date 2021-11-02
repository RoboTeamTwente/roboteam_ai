
#ifndef RTT_JOYSTICKHANDLER_H
#define RTT_JOYSTICKHANDLER_H

#include "SDL.h"
#include "SDL_joystick.h"
#include <stdio.h>
#include <iostream>
#include "JoystickState.h"
#include <roboteam_proto/RobotCommand.pb.h>

namespace rtt::input {

class JoystickHandler {
   private:
    proto::RobotCommand command;
    JoystickState joystickState;
    float robotAngle = 0.0;
    int robotId = -1;
    int dribbler_vel = 0;
    std::chrono::steady_clock::time_point id_switched_timestamp;

   public:
    JoystickHandler();
    void tick();
    void handleEvent(SDL_Event &event);
    void handleJoystickMotion(SDL_Event &event);
    void handleJoystickButton(SDL_Event &event);
    void handleJoystickHat(SDL_Event &event);
    proto::RobotCommand getCommand();

    void updateVelocity();
    void updateOrientation();
    void doKick();
    void doChip();
    void tuneDribbler();
    void toggleDribbler();
    void changeRobotID();
    JoystickState getJoystickState();
};

}  // namespace rtt::input

#endif  // RTT_JOYSTICKHANDLER_H
