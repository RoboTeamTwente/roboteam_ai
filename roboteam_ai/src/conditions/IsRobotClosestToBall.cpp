//
// Created by robzelluf on 10/18/18.
//

//TODO: get/make function get_robot_closest_to_point

#include "IsRobotClosestToBall.h"

namespace rtt {
namespace ai{

    IsRobotClosestToBall::IsRobotClosestToBall(std::string name, bt::Blackboard::Ptr blackboard) : Condition(name, blackboard) {

    }

    bt::Node::Status IsRobotClosestToBall::Update() {
        roboteam_msgs::World world = World::get_world();
        int robotID = blackboard->GetInt("ROBOT_ID");
        Vector2 ballPos(world.ball.pos);
        std::vector<roboteam_msgs::WorldRobot> robots = world.us;
        boost::optional<int> robotClosestToBallPtr;

        boost::optional<int> robotClosestToBall;
        if (blackboard->HasDouble("secondsAhead")) {
            double t_ahead = blackboard->GetDouble("secondsAhead");
            Vector2 ballVel(world.ball.vel);
            ballPos = ballPos + ballVel.scale(t_ahead);
        }

        robotClosestToBallPtr = World::get_robot_closest_to_point(robots, ballPos);

        if (robotClosestToBallPtr) {
            robotClosestToBall = *robotClosestToBallPtr;
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