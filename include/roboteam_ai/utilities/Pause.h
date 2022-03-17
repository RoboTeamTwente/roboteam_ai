//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>

namespace rtt::world {
class World;
}

namespace rtt::ai {

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
    void haltRobots(rtt::world::World const* data);
    void setPause(bool set);
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_PAUSE_H
