//
// Created by thijs on 21-3-19.
//

#ifndef ROBOTEAM_AI_WORLDDATAA_H
#define ROBOTEAM_AI_WORLDDATAA_H

#include "roboteam_msgs/World.h"

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"

#include "Robot.h"
#include "Ball.h"

namespace rtt {
namespace ai {
namespace world {

enum WhichRobots : short {
  OUR_ROBOTS,
  THEIR_ROBOTS,
  ALL_ROBOTS
};

class WorldData {
    public:
        WorldData() = default;
        explicit WorldData(const roboteam_msgs::World &copy)
                :time(copy.time), ball(copy.ball) {
            for (auto &robot : copy.us) {
                us.emplace_back(robot, Robot::Team::us);
            }
            for (auto &robot : copy.them) {
                them.emplace_back(robot, Robot::Team::them);
            }
        }
        double time = 0.0;
        std::vector<Robot> us;
        std::vector<Robot> them;
        Ball ball;
};

class WorldBuffer {
    private:
        WorldData* worldBuffer;
        unsigned int size;
        int lastIndex;
    public:
        explicit WorldBuffer(unsigned int size = 20) {
            WorldBuffer::size = size;
            worldBuffer = new WorldData[size];
            lastIndex = 0;
        }

        ~WorldBuffer() {
            delete[] worldBuffer;
        }

        void addNewWorld(const WorldData &world) {
            worldBuffer[lastIndex --] = world;
            if (lastIndex < 0) lastIndex = size - 1;
        }

        void addNewWorld(const roboteam_msgs::World &worldMsg) {
            WorldData worldData = WorldData(worldMsg);
            addNewWorld(worldData);
        }

        const WorldData &getPreviousWorld(const unsigned int worldsBack) {
            unsigned int location = lastIndex + worldsBack + 1;
            location %= size;
            return worldBuffer[location];
        }
};

}
}
}

#endif //ROBOTEAM_AI_WORLDDATA_H
