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
#include "roboteam_utils/constants.h"
#include "roboteam_msgs/World.h"
#include "roboteam_msgs/GeometryData.h"

namespace rtt {
namespace ai {

class World {
    private:
        static roboteam_msgs::World world;
    public:
        static roboteam_msgs::WorldBall getBall();
        static bool didReceiveFirstWorld;
        static std::shared_ptr<roboteam_msgs::WorldRobot> getRobotForId(unsigned int id, bool ourTeam);
        static const roboteam_msgs::World &get_world();
        static void set_world(roboteam_msgs::World world);
        static std::shared_ptr<int> get_robot_closest_to_point(std::vector<roboteam_msgs::WorldRobot> robots,
                const Vector2 &point);
        static bool bot_has_ball(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::WorldBall &ball);
        static std::vector<roboteam_msgs::WorldRobot> getAllRobots();
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_WORLD_H
