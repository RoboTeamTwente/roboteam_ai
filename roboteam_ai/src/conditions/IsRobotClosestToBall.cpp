//
// Created by robzelluf on 10/18/18.
//

#include "IsRobotClosestToBall.h"

namespace rtt {
namespace ai{

    IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

    }

    bt::Node::Status IsRobotClosestToBall::update() {
        robot = getRobotFromProperties(properties);
        roboteam_msgs::World world = World::get_world();
        Vector2 ballPos(world.ball.pos);
        std::vector<roboteam_msgs::WorldRobot> robots = world.us;
        std::shared_ptr<int> robotClosestToBallPtr;

        int robotClosestToBall;
        if (properties->hasDouble("secondsAhead")) {
            double t_ahead = properties->getDouble("secondsAhead");
            Vector2 ballVel(world.ball.vel);
            ballPos = ballPos + ballVel.scale(t_ahead);
        }

        robotClosestToBallPtr = World::get_robot_closest_to_point(robots, ballPos);

        if (robotClosestToBallPtr) {
            robotClosestToBall = *robotClosestToBallPtr;
            if (robot->id == robotClosestToBall) {
                return Status::Success;
            } else {
                return Status::Failure;
            }
        } else {
            return Status::Failure;
        }
    }
}
}