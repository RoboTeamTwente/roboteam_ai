//
// Created by baris on 14-2-19.
//

#ifndef ROBOTEAM_AI_JOYSTICKDEMO_H
#define ROBOTEAM_AI_JOYSTICKDEMO_H

#include <mutex>
#include <set>
#include <vector>
#include "roboteam_proto/DemoRobot.pb.h"

namespace demo {
class JoystickDemo {
   private:
    static std::set<int> demoRobots;
    static std::mutex demoLock;

   public:
    static bool isDemo();
    static void demoLoop(proto::DemoRobot msg);
    static std::set<int> getDemoRobots();
    static bool checkIfDemoSafe(int ID);
};
}  // namespace demo

#endif  // ROBOTEAM_AI_JOYSTICKDEMO_H
