//
// Created by thijs on 19-3-19.
//

#ifndef ROBOTEAM_AI_WORLD_H
#define ROBOTEAM_AI_WORLD_H

#include <utility>
#include <mutex>
#include <thread>

#include "roboteam_utils/Vector2.h"
#include "roboteam_utils/Angle.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/WorldRobot.h"
#include "roboteam_msgs/GeometryData.h"

#include "../utilities/Constants.h"
#include "WorldData.h"
#include "FutureWorld.h"
#include "History.h"

namespace rtt {
namespace ai {
namespace world {

class World {
    public:
        using RobotPtr = std::shared_ptr<Robot>;
        using BallPtr = std::shared_ptr<Ball>;
        using WorldDataPtr = std::shared_ptr<WorldData>;

    private:
        roboteam_msgs::World worldMsg;
        // Always use worldDataPtr in functions!!!
        WorldData worldData;
        WorldDataPtr worldDataPtr;

        History history;
        FutureWorld futureWorld;

        std::mutex worldMutex;
        std::mutex worldMsgMutex;

        const roboteam_msgs::World makeWorldMsg();
        const roboteam_msgs::WorldRobot makeWorldRobotMsg(const Robot &robot);
        const roboteam_msgs::WorldBall makeWorldBallMsg(const Ball &ball);

        Robot getRobotClosestToPoint(const Vector2 &point, std::vector<Robot> robots);
    public:
        void updateWorld(const roboteam_msgs::World &world);
        bool weHaveRobots();

        void setWorldData(WorldDataPtr &setWorldDataPtr);

        const roboteam_msgs::World &getWorldMsg();
        const roboteam_msgs::WorldBall &getBallMsg();

        const WorldData getWorld();
        const WorldData getPreviousWorld();
        double timeDifference();
        const BallPtr getBall();
        const RobotPtr getRobotForId(int id, bool ourTeam = true);
        const std::vector<world::Robot> getRobotsForIds(std::vector<int> ids, bool ourTeam = true);

    const std::vector<Robot> getAllRobots();
        const std::vector<Robot> getUs();
        const std::vector<Robot> getThem();

    Robot getRobotClosestToPoint(const Vector2 &point, std::vector<int>robotIds, bool ourTeam);

    Robot getRobotClosestToPoint(const Vector2 &point, WhichRobots whichRobots = ALL_ROBOTS);
        Robot getRobotClosestToRobot(const RobotPtr &robot, WhichRobots whichRobots = ALL_ROBOTS);
        Robot getRobotClosestToRobot(int id, bool ourTeam, WhichRobots whichRobots = ALL_ROBOTS);
        Robot getRobotClosestToBall(WhichRobots whichRobots = ALL_ROBOTS);

        bool robotHasBall(int id, bool ourTeam, double maxDist = Constants::MAX_BALL_RANGE());
        bool ourRobotHasBall(int id, double maxDist = Constants::MAX_BALL_RANGE());
        bool theirRobotHasBall(int id, double maxDist = Constants::MAX_BALL_RANGE());
        const RobotPtr whichRobotHasBall(WhichRobots whichRobots = ALL_ROBOTS);

        const WorldData getFutureWorld(double time);
        const RobotPtr getFutureRobot(int id, bool ourTeam, double time);
        const Robot getFutureRobot(const Robot &robot, double time);
        const RobotPtr getFutureRobot(const RobotPtr &robot, double time);
        const BallPtr getFutureBall(double time);
};

extern World worldObj;
extern World* world;

} //world
} //ai
} //rtt


#endif //ROBOTEAM_AI_WORLD_H
