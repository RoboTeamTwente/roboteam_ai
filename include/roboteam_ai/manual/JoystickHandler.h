
#ifndef RTT_JOYSTICKHANDLER_H
#define RTT_JOYSTICKHANDLER_H

#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <stdio.h>
#include <iostream>
#include "JoystickState.h"
#include "roboteam_proto/RobotCommand.pb.h"

namespace rtt {
namespace input {

class JoystickHandler {
   private:
    proto::RobotCommand command;
    JoystickState joystickState;
    float robotAngle = 0.0;
    int robotId = -1;

   public:
    JoystickHandler();
    void tick();
    void handleEvent(SDL_Event &event);
    void handleJoystickMotion(SDL_Event &event);
    void handleJoystickButton(SDL_Event &event);
    proto::RobotCommand getCommand();

    void updateVelocity();
    void updateOrientation();
    void doKick();
    void doChip();
    void toggleDribbler();
    void changeRobotID();
    JoystickState getJoystickState();
};

}  // namespace input
}  // namespace rtt

#endif  // RTT_JOYSTICKHANDLER_H
