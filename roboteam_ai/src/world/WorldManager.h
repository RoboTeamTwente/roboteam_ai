//
// Created by thijs on 21-3-19.
//

#ifndef ROBOTEAM_AI_WORLDMANAGER_H
#define ROBOTEAM_AI_WORLDMANAGER_H

namespace rtt {
namespace ai {

namespace io {
    class IOManager;
} // io

namespace world {
class WorldManager {
private:
    io::IOManager* IOManager;

public:
    WorldManager() = default;
    void setup();
    void loop();
};

} // world
} // ai
} // rtt

#endif //ROBOTEAM_AI_WORLDMANAGER_H
