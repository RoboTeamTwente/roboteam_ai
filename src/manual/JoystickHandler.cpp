//
// Created by luukkn on 23-10-19.
//
#include <iostream>
#include "include/roboteam_ai/manual/JoystickHandler.h"
#include <SDL.h>

namespace rtt {
namespace input {

JoystickHandler::JoystickHandler() {
    std::cout << "[JoystickHandler] New JoystickHandler" << std::endl;
    command.set_chip_kick_forced(true);
};

void JoystickHandler::tick(){
    updateVelocity();
    updateOrientation();
    doKick();
    doChip();
    changeRobotID();
    toggleDribbler();

//            std::cout << command.id();
//            std::cout << " vel=" << command.vel();
//            std::cout << " w=" << command.w();
//            std::cout << " use_angle=" << command.use_angle();
//            std::cout << std::endl;

}

/** Receives event data and checks the event type.
 * Then updates internal state accordingly.
 * */
void JoystickHandler::handleEvent(SDL_Event& event) {
    switch (event.type) {
        case SDL_JOYAXISMOTION: /* Handle Axis motion*/
            handleJoystickMotion(event);
            break;
        case SDL_JOYBUTTONUP: /* Handle Button unpressing*/
            handleJoystickButton(event);
            break;
        case SDL_JOYBUTTONDOWN: /* Handle Button pressing*/
            handleJoystickButton(event);
            break;
    }
}

void JoystickHandler::changeRobotID() {
    if(joystickState.back){
        if(joystickState.bumperLeft){
            if (0 < robotId) {
                joystickState.dpadLeft = false;
                robotId--;
                std::cout << "[JoystickHandler][changeRobotId] Switched to robot " << robotId << std::endl;
            }
            else
                std::cout << "[JoystickHandler][changeRobotId] No robots with lower ID available" << std::endl;
        }
        if(joystickState.bumperRight){
            if (robotId < 16) {
                joystickState.dpadRight = false;
                robotId++;
                std::cout << "[JoystickHandler][changeRobotId] Switched to robot " << robotId << std::endl;
            }
            else
                std::cout << "[JoystickHandler][changeRobotId] No robots with higher ID available" << std::endl;
        }
        command.set_id(robotId);
    }
}

void JoystickHandler::doKick(){
    if (joystickState.A) {
        command.set_kicker(true);
        command.set_chip_kick_vel(4.0);
        joystickState.A = false;
    }
    else if (joystickState.B) {
        command.set_kicker(true);
        command.set_chip_kick_vel(8.0);
        joystickState.B = false;
    }
    else {
        command.set_kicker(false);
    }
}

void JoystickHandler::doChip(){
    if (joystickState.Y) {
        command.set_chipper(true);
        command.set_chip_kick_vel(4.0);
        joystickState.Y = false;
    }
    else if (joystickState.X) {
        command.set_chipper(true);
        command.set_chip_kick_vel(8.0);
        joystickState.X = false;
    }
    else {
        command.set_chipper(false);
    }
}

void JoystickHandler::toggleDribbler(){
    if (joystickState.bumperRight) {
        joystickState.bumperRight = false;
        if(0 < command.dribbler()) {
            command.set_dribbler(0);
        }else{
            command.set_dribbler(16);
        }
    }
}
void JoystickHandler::updateOrientation() {
    /* Robot angle */
    command.set_use_angle(true);
    float dAngle = -joystickState.stickRight.x / 32768.0;
    robotAngle += dAngle * 0.1;
    while(M_PI < robotAngle) robotAngle -= 2 * M_PI;
    while(robotAngle < -M_PI) robotAngle += 2 * M_PI;
    command.set_w(robotAngle);
}

void JoystickHandler::updateVelocity(){
    /* Robot velocity, value 1 for mutable vel is achieved by dividing by 32768 instead of 22000*/

    rtt::Vector2 driveVector = joystickState.stickLeft.rotate(-robotAngle)/22000;
    command.mutable_vel()->set_y(-driveVector.x);
    command.mutable_vel()->set_x(-driveVector.y);
}


/* Processes the joystick motion */
void JoystickHandler::handleJoystickMotion(SDL_Event &event){
    /* Check if values are outside of the deadzone */
    if (-6000 < event.jaxis.value && event.jaxis.value < 6000) {
        event.jaxis.value = 0;
    }
    switch(event.jaxis.axis){
        case 0 : joystickState.stickLeft.x = event.jaxis.value; break;
        case 1 : joystickState.stickLeft.y = event.jaxis.value; break;
        case 2 : joystickState.triggerLeft = event.jaxis.value; break;
        case 3 : joystickState.stickRight.x = event.jaxis.value; break;
        case 4 : joystickState.stickRight.y = event.jaxis.value; break;
        case 5 : joystickState.triggerRight = event.jaxis.value; break;
    }
}

/* Maps buttons*/
void JoystickHandler::handleJoystickButton(SDL_Event &event) {
    bool button_State = (int) event.jbutton.state == 1;
    switch(event.jbutton.button){
        case  0 : joystickState.A = button_State; break;
        case  1 : joystickState.B = button_State; break;
        case  2 : joystickState.X = button_State; break;
        case  3 : joystickState.Y = button_State; break;
        case  4 : joystickState.bumperLeft = button_State; break;
        case  5 : joystickState.bumperRight = button_State; break;
        case  6 : joystickState.back = button_State; break;
        case  7 : joystickState.start = button_State; break;
        case  8 : joystickState.XBOX = button_State; break;
        case  9 : joystickState.stickLeftBtn = button_State; break;
        case 10 : joystickState.stickRightBtn = button_State; break;
        case 11 : joystickState.dpadLeft = button_State; break;
        case 12 : joystickState.dpadRight = button_State; break;
        case 13 : joystickState.dpadUp = button_State; break;
        case 14 : joystickState.dpadDown = button_State; break;
    }
}

roboteam_proto::RobotCommand JoystickHandler::getCommand(){
    return command;
}
JoystickState JoystickHandler::getJoystickState(){
    return joystickState;
}


} // namespace input
} // namespace rtt