//
// Created by luukkn on 23-10-19.
//
#include "manual/JoystickHandler.h"

#include <roboteam_utils/Print.h>

namespace rtt {
namespace input {

JoystickHandler::JoystickHandler() {
    RTT_INFO("Created new JoystickHandler")
    id_switched_timestamp = std::chrono::steady_clock::now();
    command.waitForBall = false;
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
        if (msFromPreviousSwitch < 100) return;

        if (joystickState.dpadLeft) {
            if (0 < robotId) {
                joystickState.dpadLeft = false;
                robotId--;
                dribbler_vel = 0;
                RTT_INFO("Joystick changed robot ID to: ", robotId)
            } else
                RTT_INFO("Joystick cannot change robot ID lower than: ", robotId)
        }
        if (joystickState.dpadRight) {
            if (robotId < 16) {
                joystickState.dpadRight = false;
                robotId++;
                dribbler_vel = 0;
                RTT_INFO("Joystick changed robot ID to: ", robotId)
            } else
                RTT_INFO("Joystick cannot change robot ID higher than: ", robotId)
        }

        command.id = robotId;
        id_switched_timestamp = std::chrono::steady_clock::now();
    }
}

void JoystickHandler::doKick() {
    static int counter = 0;
    if (joystickState.A) {
        command.kickType = KickType::KICK;
        command.kickSpeed = 3.5;
        counter++;
        // Loop 5 times to be sure that the robot actually kicks
        if (counter >= 5) {
            joystickState.A = false;
            counter = 0;
        }
    } else if (joystickState.B) {
        command.kickType = KickType::KICK;
        command.kickSpeed = 5;
        counter++;
        if (counter >= 5) {
            joystickState.B = false;
            counter = 0;
        }
    } else {
        command.kickSpeed = 0.0;
    }
}

void JoystickHandler::doChip() {
    if (joystickState.Y) {
        command.kickType = KickType::CHIP;
        command.kickSpeed = 1.0;
        joystickState.Y = false;
    } else if (joystickState.X) {
        command.kickType = KickType::CHIP;
        command.kickSpeed = 4;
        joystickState.X = false;
    }
}

void JoystickHandler::toggleDribbler() {
    if (joystickState.bumperRight) {
        joystickState.bumperRight = false;
        if (0 < command.dribblerSpeed) {
            command.dribblerSpeed = 0;
        } else {
            command.dribblerSpeed = 1;
        }
    }
}
void JoystickHandler::updateOrientation() {
    /* Robot angle */
    command.useAngularVelocity = false;
    float dAngle = -joystickState.stickRight.x / 32768.0;
    robotAngle += dAngle * 0.05;
    command.targetAngle = Angle(robotAngle);
}

void JoystickHandler::updateVelocity() {
    /* Robot velocity, value 1 for mutable vel is achieved by dividing by 32768 instead of 22000*/

    rtt::Vector2 driveVector = joystickState.stickLeft.rotate(-robotAngle) / 32768.0;
    command.velocity.y = -driveVector.x;
    command.velocity.x = -driveVector.y;
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
        case 4:;
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
        case 9:;
            break;
        case 10:;
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

    joystickState.dpadRight = (value & 2) > 0;
    joystickState.dpadLeft = (value & 8) > 0;
}

/* Sets dribber speed */
void JoystickHandler::tuneDribbler() {
    if (joystickState.triggerLeft > 32766) dribbler_vel -= 0.03;

    if (joystickState.triggerRight > 32766) dribbler_vel += 0.03;

    if (dribbler_vel < 0) dribbler_vel = 0;
    if (1 < dribbler_vel) dribbler_vel = 1;

    command.dribblerSpeed = dribbler_vel;
}
rtt::RobotCommand JoystickHandler::getCommand() { return command; }
JoystickState JoystickHandler::getJoystickState() { return joystickState; }

}  // namespace input
}  // namespace rtt