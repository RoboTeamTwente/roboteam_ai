//
// Created by baris on 14-2-19.
//

#include <algorithm>
#include "JoystickDemo.h"

namespace demo {

std::vector<int> JoystickDemo::demoRobots;
std::mutex JoystickDemo::demoLock;

/// Some indication of if there is a demo going
bool JoystickDemo::isDemo() {
    std::lock_guard<std::mutex> lock(demoLock);
    return (! demoRobots.empty());

}

/// Tool to check if demo stuff should happen every loop
void JoystickDemo::demoLoop() {

    // Check the robots we have to block

    // Let go of the robots we should

    // Update the vector


}

/// Returns the robots that are used in the demo
/// currently probably with the joystick
std::vector<int> JoystickDemo::getDemoRobots() {
    std::lock_guard<std::mutex> lock(demoLock);
    return demoRobots;
}

/// Take care of demo behaviour
void JoystickDemo::runDemoUtils() {

}
void JoystickDemo::checkROS() {

}
bool JoystickDemo::checkIfDemoSafe(int ID) {
    std::lock_guard<std::mutex> lock(demoLock);
    // True if not in the vector
    return (std::find(demoRobots.begin(), demoRobots.end(), ID) == demoRobots.end());
}
}
