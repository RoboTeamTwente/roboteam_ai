//
// Created by thijs on 21-3-19.
//

#ifndef ROBOTEAM_AI_WORLDMANAGER_H
#define ROBOTEAM_AI_WORLDMANAGER_H

namespace rtt {
namespace ai {
namespace io {
    class IOManager;
}

namespace world {
class WorldManager {
private:
    io::IOManager* IOManager;

public:
    WorldManager() = default;
    void setup();
    void loop();
};

}
}
}

#endif //ROBOTEAM_AI_WORLDMANAGER_H
