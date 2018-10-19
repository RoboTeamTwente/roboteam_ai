//
// Created by robzelluf on 10/18/18.
//

//TODO: get/make function get_robot_closest_to_point

#include "IsRobotClosestToBall.h"

namespace rtt {
namespace ai{
    bt::Node::Status IsRobotClosestToBall::Update() {
        roboteam_msgs::World world = LastWorld::get();
        int robotID = blackboard->GetInt("ROBOT_ID");
        Vector2 ballPos(world.ball.pos);
        std::vector<roboteam_msgs::WorldRobot> robots world.us;

        boost::optional<int> robotClosestToBallPtr;
        if (blackboard->HasDouble("secondsAhead")) {
            double t_ahead = blackboard->GetDouble("secondsAhead");
            Vector2 ballVel(world.ball.vel);
            ballPos = ballPos + ballVel.scale(t_ahead);
            robotClosestToBallPtr = get_robot_closest_to_point(robots, ballPos)
        } else {
            robotClosestToBallPtr = get_robot_closest_to_point(robots, ballPos)
        }

        if (robotClosestToBallPtr) {
            int robotClosestToBall = *robotClosestToBallPtr;
            if (robotID == robotClosestToBall) {
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