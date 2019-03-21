//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>
#include <roboteam_msgs/RobotCommand.h>
#include "../world/World.h"

namespace rtt {
namespace ai {

namespace io {
class IOManager;
}

class Pause {

    private:
        world::World* world;
        static bool pause;
        static std::mutex pauseLock;
        std::shared_ptr<io::IOManager> IOManager;
    public:
        Pause();
        bool getPause();
        void haltRobots();
        void setPause(bool set);

};
}
}

#endif //ROBOTEAM_AI_PAUSE_H
