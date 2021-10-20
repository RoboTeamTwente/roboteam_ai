//
// Created by luukkn on 23-10-19.
//
#include "manual/JoystickHandler.h"

namespace rtt {
namespace input {

JoystickHandler::JoystickHandler() {
    std::cout << "[JoystickHandler] New JoystickHandler" << std::endl;
    id_switched_timestamp = std::chrono::steady_clock::now();
    command.set_chip_kick_forced(true);
}

void JoystickHandler::tick() {
    updateVelocity();
    updateOrientation();
    doKick();
    doChip();
    tuneDribbler();
    changeRobotID();
    toggleDribbler();
}

/** Receives event data and checks the event type.
 * Then updates internal state accordingly.
 * */
void JoystickHandler::handleEvent(SDL_Event &event) {
    switch (event.type) {
        case SDL_JOYAXISMOTION: /* Handle Axis motion */
            handleJoystickMotion(event);
            break;
        case SDL_JOYBUTTONUP: /* Handle Button unpressing */
            handleJoystickButton(event);
            break;
        case SDL_JOYBUTTONDOWN: /* Handle Button pressing */
            handleJoystickButton(event);
            break;
        case SDL_JOYHATMOTION: /* Handle dpad (un)pressing */
            handleJoystickHat(event);
            break;
        default:
            break;
    }
}

void JoystickHandler::changeRobotID() {
    if (joystickState.back) {

        /* Add minimal delay of 100ms between id switching to deal with button bouncing */
        int msFromPreviousSwitch = (int)duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - id_switched_timestamp).count();
        if(msFromPreviousSwitch < 100)
            return;

        if (joystickState.dpadLeft) {
            if (0 < robotId) {
                joystickState.dpadLeft = false;
                robotId--;
                dribbler_vel = 0;
                std::cout << "[JoystickHandler][changeRobotId] Switched to robot " << robotId << std::endl;
            } else
                std::cout << "[JoystickHandler][changeRobotId] No robots with lower ID available" << std::endl;
        }
        if (joystickState.dpadRight) {
            if (robotId < 16) {
                joystickState.dpadRight = false;
                robotId++;
                dribbler_vel = 0;
                std::cout << "[JoystickHandler][changeRobotId] Switched to robot " << robotId << std::endl;
            } else
                std::cout << "[JoystickHandler][changeRobotId] No robots with higher ID available" << std::endl;
        }

        command.set_id(robotId);
        id_switched_timestamp = std::chrono::steady_clock::now();
    }
}

void JoystickHandler::doKick() {
    if (joystickState.A) {
        command.set_kicker(true);
        command.set_chip_kick_vel(1.0);
        joystickState.A = false;
    } else if (joystickState.B) {
        command.set_kicker(true);
        command.set_chip_kick_vel(6.5);
        joystickState.B = false;
    } else {
        command.set_kicker(false);
    }
}

void JoystickHandler::doChip() {
    if (joystickState.Y) {
        command.set_chipper(true);
        command.set_chip_kick_vel(1.0);
        joystickState.Y = false;
    } else if (joystickState.X) {
        command.set_chipper(true);
        command.set_chip_kick_vel(4);
        joystickState.X = false;
    } else {
        command.set_chipper(false);
    }
}

void JoystickHandler::toggleDribbler() {
    if (joystickState.bumperRight) {
        joystickState.bumperRight = false;
        if (0 < command.dribbler()) {
            command.set_dribbler(0);
        } else {
            command.set_dribbler(10);
        }
    }
}
void JoystickHandler::updateOrientation() {
    /* Robot angle */
    command.set_use_angle(true);
    float dAngle = -joystickState.stickRight.x / 32768.0;
    robotAngle += dAngle * 0.05;
    while (M_PI < robotAngle) robotAngle -= 2 * M_PI;
    while (robotAngle < -M_PI) robotAngle += 2 * M_PI;
    command.set_w(robotAngle);
}

void JoystickHandler::updateVelocity() {
    /* Robot velocity, value 1 for mutable vel is achieved by dividing by 32768 instead of 22000*/

    rtt::Vector2 driveVector = joystickState.stickLeft.rotate(-robotAngle) / 32768.0;
    command.mutable_vel()->set_y(-driveVector.x);
    command.mutable_vel()->set_x(-driveVector.y);
}

/* Processes the joystick motion */
void JoystickHandler::handleJoystickMotion(SDL_Event &event) {
    /* Check if values are outside of the deadzone */
    if (-10000 < event.jaxis.value && event.jaxis.value < 10000) {
        event.jaxis.value = 0;
    }
    switch (event.jaxis.axis) {
        case 0:
            joystickState.stickLeft.x = event.jaxis.value;
            break;
        case 1:
            joystickState.stickLeft.y = event.jaxis.value;
            break;
        case 2:
            joystickState.triggerLeft = event.jaxis.value;
            break;
        case 3:
            joystickState.stickRight.x = event.jaxis.value;
            break;
        case 4:
            joystickState.stickRight.y = event.jaxis.value;
            break;
        case 5:
            joystickState.triggerRight = event.jaxis.value;
            break;
        default:
            break;
    }
}

/* Maps buttons*/
void JoystickHandler::handleJoystickButton(SDL_Event &event) {
    bool button_State = (int)event.jbutton.state == 1;

    switch (event.jbutton.button) {
        case 0:
            joystickState.A = button_State;
            break;
        case 1:
            joystickState.B = button_State;
            break;
        case 2:
            joystickState.X = button_State;
            break;
        case 3:
            joystickState.Y = button_State;
            break;
        case 4:
            joystickState.bumperLeft = button_State;
            break;
        case 5:
            joystickState.bumperRight = button_State;
            break;
        case 6:
            joystickState.back = button_State;
            break;
        case 7:
            joystickState.start = button_State;
            break;
        case 8:
            joystickState.XBOX = button_State;
            break;
        case 9:
            joystickState.stickLeftBtn = button_State;
            break;
        case 10:
            joystickState.stickRightBtn = button_State;
            break;
        default:
            break;
    }
}

void JoystickHandler::handleJoystickHat(SDL_Event &event) {
    /* This function deals with the dpad of the joystick, which is apparently a Hat event
     * event.jhat.value indicates the directions(s) that are pressed on the dpad.
     * 1 = up, 2 = right, 4 = down, 8 = left. Also, 3 = up+right, 12 = down+left, etc. It's stuff with bits.
     */

    uint32_t value = event.jhat.value;

    joystickState.dpadUp    = (value & 1) > 0;
    joystickState.dpadRight = (value & 2) > 0;
    joystickState.dpadDown  = (value & 4) > 0;
    joystickState.dpadLeft  = (value & 8) > 0;
}

/* Sets dribber speed */
void JoystickHandler::tuneDribbler() {
    if (joystickState.triggerLeft > 32766) dribbler_vel -= 1;

    if (joystickState.triggerRight > 32766) dribbler_vel += 1;

    if (dribbler_vel < 0) dribbler_vel = 0;
    if (31 < dribbler_vel) dribbler_vel = 31;

    command.set_dribbler(dribbler_vel);
}
proto::RobotCommand JoystickHandler::getCommand() { return command; }
JoystickState JoystickHandler::getJoystickState() { return joystickState; }

}  // namespace input
}  // namespace rtt