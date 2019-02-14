//
// Created by baris on 14-2-19.
//

#ifndef ROBOTEAM_AI_JOYSTICKDEMO_H
#define ROBOTEAM_AI_JOYSTICKDEMO_H

#include <vector>
#include <mutex>
namespace demo {
class JoystickDemo {

    private:
        static std::vector<int> demoRobots;
        static std::mutex demoLock;
        static void runDemoUtils();
        static void checkROS();


    public:
        static bool isDemo();
        static void demoLoop();
        static std::vector<int> getDemoRobots();
        static bool checkIfDemoSafe(int ID);






};
}

#endif //ROBOTEAM_AI_JOYSTICKDEMO_H
