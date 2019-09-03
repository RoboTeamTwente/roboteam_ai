//
// Created by baris on 14-2-19.
//

#include <algorithm>
#include "demo/JoystickDemo.h"
#include "utilities/Pause.h"

namespace demo {

std::set<int> JoystickDemo::demoRobots;
std::mutex JoystickDemo::demoLock;

/// Some indication of if there is a demo going
bool JoystickDemo::isDemo() {
    std::lock_guard<std::mutex> lock(demoLock);
    return (! demoRobots.empty());

}

/// Tool to check if demo stuff should happen every loop
void JoystickDemo::demoLoop(roboteam_proto::DemoRobot msg) {
    rtt::ai::Pause* pause{};

    std::lock_guard<std::mutex> lock(demoLock);
    if (msg.reserve() && msg.halt() == 0) {
        demoRobots.insert(msg.id());
    }

    else if (! msg.reserve()) {
        demoRobots.erase(msg.id());
    }

    if (msg.halt() == 1) {
        pause->setPause(true);
        pause->haltRobots();
    }
    else if (msg.halt() == 2) {
        pause->setPause(false);
    }

}

/// Returns the robots that are used in the demo
/// currently probably with the joystick
std::set<int> JoystickDemo::getDemoRobots() {
    std::lock_guard<std::mutex> lock(demoLock);
    return demoRobots;
}

/// For use outside this class
bool JoystickDemo::checkIfDemoSafe(int ID) {
    std::lock_guard<std::mutex> lock(demoLock);
    // True if not in the vector
    return (demoRobots.find(ID) == demoRobots.end());
}
}
