//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>
#include "../io/IOManager.h"
#include <roboteam_msgs/RobotCommand.h>
#include "World.h"

namespace rtt {
namespace pause {

class Pause {

    private:
        static bool pause;
        static std::mutex pauseLock;
        rtt::ai::io::IOManager IOManager = rtt::ai::io::IOManager(false,true);
    public:
        bool getPause();
        void haltRobots();
        void setPause(bool set);

};
}
}

#endif //ROBOTEAM_AI_PAUSE_H
