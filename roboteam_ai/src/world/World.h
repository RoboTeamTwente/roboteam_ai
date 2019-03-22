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

class ProcessedWorld;
class World {
    private:
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;

        roboteam_msgs::World worldMsg;
        ProcessedWorld* processedWorld;
        WorldData worldData;

        std::mutex worldMutex;
        std::mutex worldMsgMutex;

    public:
        enum WhichRobots {
          OUR_ROBOTS,
          THEIR_ROBOTS,
          ALL_ROBOTS
        };

        bool weHaveRobots();
        void setWorld(const roboteam_msgs::World &world);

        const roboteam_msgs::World &getWorldMsg();
        const roboteam_msgs::WorldBall &getBallMsg();

        const WorldData &getWorld();
        BallPtr getBall();
        RobotPtr getRobotForId(int id, bool ourTeam = true);
        std::vector<RobotPtr> getAllRobots();

        RobotPtr getRobotClosestToPoint(const Vector2 &point, WhichRobots whichRobots = ALL_ROBOTS);
        RobotPtr getRobotClosestToRobot(int id, bool ourTeam, WhichRobots whichRobots = ALL_ROBOTS);
        RobotPtr getRobotClosestToBall(WhichRobots whichRobots = ALL_ROBOTS);

        bool robotHasBall(int id, bool ourTeam, double maxDist = Constants::MAX_BALL_RANGE());
        bool ourRobotHasBall(int id, double maxDist = Constants::MAX_BALL_RANGE());
        bool theirRobotHasBall(int id, double maxDist = Constants::MAX_BALL_RANGE());
        const WorldData getFutureWorld(double time);
};

World* world;

} //world
} //ai
} //rtt


#endif //ROBOTEAM_AI_WORLD_H
