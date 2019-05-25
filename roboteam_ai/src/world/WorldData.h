//
// Created by thijs on 21-3-19.
//

#ifndef ROBOTEAM_AI_WORLDDATA_H
#define ROBOTEAM_AI_WORLDDATA_H

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
    private:
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;
    public:
        WorldData() = default;
        explicit WorldData(const roboteam_msgs::World &copy)
                :time(copy.time) {
            for (auto &robot : copy.us) {
                RobotPtr r = std::make_shared<Robot>(Robot(robot, Robot::Team::us, 3));
                us.emplace_back(r);
            }

            for (auto &robot : copy.them) {
                RobotPtr r = std::make_shared<Robot>(Robot(robot, Robot::Team::them, 3));
                them.emplace_back(r);
            }
            ball = std::make_shared<Ball>(Ball(copy.ball));
        }
        explicit WorldData(const std::vector<RobotPtr> &copyUs, const std::vector<RobotPtr> &copyThem,
                const BallPtr &copyBall, double time)
                :time(time) {
            for (auto &robot : copyUs) {
                us.emplace_back(robot->copy());
            }
            for (auto &robot : copyThem) {
                them.emplace_back(robot->copy());
            }
            if (copyBall) ball = copyBall->copy();
        }
        explicit WorldData(const WorldDataPtr &worldDataPtr)
                :WorldData(*worldDataPtr) { }
        WorldData(const WorldData &worldData)
                :WorldData(worldData.us, worldData.them, worldData.ball, worldData.time) { }

        double time = 0.0;
        std::vector<RobotPtr> us;
        std::vector<RobotPtr> them;
        BallPtr ball;
};

class WorldBuffer {
    private:
        WorldData* worldBuffer;
        int size;
        int lastIndex;
        int amountOfWorlds;
    public:
        explicit WorldBuffer(unsigned int size = 20)
                :size(size), amountOfWorlds(0) {
            worldBuffer = new WorldData[size];
            lastIndex = 0;
        }

        ~WorldBuffer() {
            delete[] worldBuffer;
        }

        void addNewWorld(const WorldData &world) {
            lastIndex ++;
            amountOfWorlds ++;
            if (lastIndex >= size) lastIndex = 0;
            worldBuffer[lastIndex] = world;
        }

        const WorldData getPreviousWorld(const int worldsBack) {
            if (worldsBack > amountOfWorlds - 1) return getPreviousWorld(amountOfWorlds - 1);

            int location = lastIndex - worldsBack;
            location %= size;
            if (location < 0) location += 20;
            return worldBuffer[location];
        }
};

}
}
}

#endif //ROBOTEAM_AI_WORLDDATA_H
