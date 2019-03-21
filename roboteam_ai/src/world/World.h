//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_WORLD_H
#define ROBOTEAM_AI_WORLD_H

#include <utility>
#include <mutex>
#include <thread>
#include <roboteam_ai/src/control/ControlUtils.h>
#include "../utilities/Constants.h"
#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryData.h"
#include "WorldData.h"

namespace rtt {
namespace ai {
namespace world {

class World {
    private:
        enum WhichRobots {
          OUR_ROBOTS,
          THEIR_ROBOTS,
          ALL_ROBOTS
        };

        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;
        roboteam_msgs::World worldMsg;
        WorldData worldData;

        //TODO: make threadsafe
        std::mutex worldMutex;
        std::mutex worldMsgMutex;

    public:
        void setWorld(const roboteam_msgs::World &world);

        const roboteam_msgs::World &getWorldMsg();
        const roboteam_msgs::WorldBall &getBallMsg();

        const WorldData &getWorld();
        BallPtr getBall();
        RobotPtr getRobotForId(int id, bool ourTeam = true);
        std::vector<RobotPtr> getAllRobots();
        RobotPtr getRobotClosestToPoint(const Vector2& point, WhichRobots whichRobots);


};

World* world;

} //world
} //ai
} //rtt


#endif //ROBOTEAM_AI_WORLD_H
