/*
 * World.h
 * This class maintains the world object, which is a message that contains:
 *    - the location of our robots (us)
 *    - the locations of their robots (them)
 *    - the location of the ball
 *  This class also provides helper functions
 *    - Getting robots for ID
 *    - Getting robots by team
 */

#ifndef ROBOTEAM_AI_WORLD_H
#define ROBOTEAM_AI_WORLD_H


#include "roboteam_utils/Vector2.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryData.h"
#include <mutex>
#include <thread>
#include "Constants.h"


namespace rtt {
namespace ai {

typedef std::shared_ptr<roboteam_msgs::WorldRobot> robotPtr;

class World {
private:
    static roboteam_msgs::World world;
    static std::vector<std::pair<roboteam_msgs::World, double>> futureWorlds;
    static std::mutex worldMutex;
    static std::map<int,double> OurBotsBall, TheirBotsBall;
    static double findBallDist(roboteam_msgs::WorldRobot &bot, roboteam_msgs::WorldBall &ball);
    static void updateBallPossession(roboteam_msgs::World &_world);
    static roboteam_msgs::WorldBall updateBallPosition(roboteam_msgs::World _world);
public:
    static void set_world(roboteam_msgs::World world);
    static const roboteam_msgs::World& get_world();
    static std::shared_ptr<roboteam_msgs::WorldBall> getBall();
    static bool didReceiveFirstWorld;

    static robotPtr getRobotForId(unsigned int id, bool ourTeam);
    static robotPtr getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point, const int& myID, const float& t);
    static robotPtr getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point, const int& myID);
    static robotPtr getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point, const float& t);
    static robotPtr getRobotClosestToPoint(std::vector<roboteam_msgs::WorldRobot> robots, const Vector2& point);
    static robotPtr getRobotClosestToPoint(const Vector2& point, const int& myID, const float& t);
    static robotPtr getRobotClosestToPoint(const Vector2& point, const float& t);
    static robotPtr getRobotClosestToPoint(const Vector2& point, const int& myID);
    static robotPtr getRobotClosestToPoint(const Vector2 &point, const bool ourTeam);

    static bool teamHasBall(bool ourTeam);
    static bool weHaveBall();
    static bool theyHaveBall();
    static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotThatHasBall(bool ourTeam);

    static bool botHasBall(int id, bool ourTeam, double maxDistToBall = Constants::MAX_BALL_RANGE());
    static bool ourBotHasBall(int id, double maxDistToBall=Constants::MAX_BALL_RANGE());
    static bool theirBotHasBall(int id, double maxDistToBall=Constants::MAX_BALL_RANGE());
    static int whichBotHasBall(bool ourTeam);

    static std::vector<roboteam_msgs::WorldRobot> getAllRobots();
    static std::vector<roboteam_msgs::WorldRobot> getRobotsForId(std::set<unsigned int> ids, bool robotsAreOurTeam);
    static roboteam_msgs::World futureWorld(double time, double maxTimeOffset = 0.11);
    static std::pair<int, bool> getRobotClosestToBall();

    static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToBall(bool isOurTeam);
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_WORLD_H
