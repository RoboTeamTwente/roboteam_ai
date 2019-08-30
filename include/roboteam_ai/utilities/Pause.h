//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>
#include <RobotCommand.pb.h>
#include "include/roboteam_ai/world/World.h"

namespace rtt {
namespace ai {

namespace io {
class IOManager;
}

class Pause {

    private:
        static bool pause;
        static std::mutex pauseLock;
    public:
        Pause();
        bool getPause();
        void haltRobots();
        void setPause(bool set);

};
}
}

#endif //ROBOTEAM_AI_PAUSE_H
