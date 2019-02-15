//
// Created by baris on 15-2-19.
//

#ifndef ROBOTEAM_AI_PAUSE_H
#define ROBOTEAM_AI_PAUSE_H

#include <mutex>
#include <roboteam_ai/src/io/IOManager.h>
namespace rtt {

class Pause {

    private:
        static bool pause;
        static std::mutex pauseLock;
        ai::io::IOManager ioManager = ai::io::IOManager(false,true);
    public:
        bool getPause();
        void haltRobots();
        void setPause(bool set);

};
}

#endif //ROBOTEAM_AI_PAUSE_H
