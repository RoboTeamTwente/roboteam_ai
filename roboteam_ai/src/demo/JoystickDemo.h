//
// Created by baris on 14-2-19.
//

#ifndef ROBOTEAM_AI_JOYSTICKDEMO_H
#define ROBOTEAM_AI_JOYSTICKDEMO_H

#include <vector>
#include <mutex>
#include <set>
#include <roboteam_ai/src/io/IOManager.h>
namespace demo {
class JoystickDemo {

    private:
        static std::set<int> demoRobots;
        static std::mutex demoLock;
        rtt::ai::io::IOManager * IOManager;


    public:
        static bool isDemo();
        static void demoLoop(roboteam_msgs::DemoRobotPtr msg);
        static std::set<int> getDemoRobots();
        static bool checkIfDemoSafe(int ID);






};
}

#endif //ROBOTEAM_AI_JOYSTICKDEMO_H
