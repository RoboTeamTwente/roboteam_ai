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

class World {
    private:
        static roboteam_msgs::World world;
        static std::mutex worldMutex;
    public:
        static void set_world(roboteam_msgs::World world);
        static const roboteam_msgs::World &get_world();
        static std::shared_ptr<roboteam_msgs::WorldBall> getBall();
        static bool didReceiveFirstWorld;

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotForId(unsigned int id, bool ourTeam);

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPoint(
                std::vector<roboteam_msgs::WorldRobot> robots, const Vector2 &point, const int &myID, const float &t);

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPoint(
                std::vector<roboteam_msgs::WorldRobot> robots, const Vector2 &point, const int &myID);

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPoint(
                std::vector<roboteam_msgs::WorldRobot> robots, const Vector2 &point, const float &t);

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPoint(
                std::vector<roboteam_msgs::WorldRobot> robots, const Vector2 &point);

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPoint(const Vector2 &point,
                const int &myID, const float &t);

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPoint(const Vector2 &point,
                const float &t);

        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotClosestToPoint(const Vector2 &point,
                const int &myID);

        static bool robotHasBall(Vector2 robotPos, double robotOrientation, Vector2 ballPos,
                double frontDist = constants::MAX_BALL_BOUNCE_RANGE);

        static bool robotHasBall(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::WorldBall &ball,
                double frontDist = constants::MAX_BALL_BOUNCE_RANGE);

        static std::vector<roboteam_msgs::WorldRobot> getAllRobots();
        static std::vector<roboteam_msgs::WorldRobot> getRobotsForId(std::set<unsigned int> ids, bool robotsAreOurTeam);
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_WORLD_H
