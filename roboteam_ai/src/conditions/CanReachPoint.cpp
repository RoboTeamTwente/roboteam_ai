//
// Created by rolf on 22-10-18.
//

#include "CanReachPoint.hpp"
namespace rtt {
namespace ai {
CanReachPoint::CanReachPoint(std::string name, bt::Blackboard::Ptr blackboard)
        :Condition(name, blackboard) {
}

//TODO: Fix this horribly ugly implementation to a working one that uses a model not made by high-schoolers (see comment)
double CanReachPoint::estimateTimeToPoint(Vector2 currentPos, Vector2 currentVel, Vector2 targetPos) {
    Vector2 posDiff = targetPos - currentPos;
    /*
    Vector2 targetVel = posDiff.normalize().scale(posPGain);
    Vector2 velDiff = targetVel - currentVel;
    double timeToReachVel = velDiff.length() / maxAcc;
    double distanceToReachVel = velDiff.length()/2 * timeToReachVel;

    double timeToStop = decelerationDistance / (targetVel.length()/2); // deceleration time = deceleration distance / average speed during deceleration

    //ROS_INFO_STREAM("distanceToReachVel: " << distanceToReachVel << " decelerationDistance: " << decelerationDistance);

    if (posDiff.length() < (distanceToReachVel + decelerationDistance)) {
        return -1.0;
    } else {
        double distAtMaxVel = posDiff.length() - (distanceToReachVel + decelerationDistance);
        double timeAtMaxVel = distAtMaxVel / maxVel;
        return (timeToReachVel + timeToStop + timeAtMaxVel);
    }
    */

    // simple version because acceleration and deceleration calculation where not complete yet
    double averageVel = 3.0; // m/s
    ROS_INFO_STREAM("currentPos x:" << currentPos.x << " y:" << currentPos.y << "; targetPos x" << targetPos.x << " y:"
                                    << targetPos.y << " length:" << posDiff.length());
    return posDiff.length()/averageVel;
}
bt::Node::Status CanReachPoint::update() {
    int ROBOT_ID = properties->getInt("ROBOT_ID");
    double xGoal = properties->getDouble("xGoal");
    double yGoal = properties->getDouble("yGoal");

    // Wait for the first world message
    roboteam_msgs::World world = World::get_world();
    while (world.us.empty()) {
        return Status::Running;
    }

    bool our_team = true;

    Vector2 currentPos(World::getRobotForId(ROBOT_ID, our_team)->pos);
    Vector2 currentVel(World::getRobotForId(ROBOT_ID, our_team)->vel);
    Vector2 targetPos(xGoal, yGoal);

    if (properties->hasString("whichTeam") && properties->getString("whichTeam") == "them") {
        currentPos = world.them.at(ROBOT_ID).pos;
        currentVel = world.them.at(ROBOT_ID).vel;
        ROS_INFO_STREAM("using their team");
    }

    double estimatedTimeToPoint = estimateTimeToPoint(currentPos, currentVel, targetPos);

    ROS_INFO_STREAM("estimatedTimeToPoint: " << estimatedTimeToPoint);

    if (estimatedTimeToPoint < 0.0) {
        // ROS_INFO_STREAM("hmm, distance too short");
        return Status::Failure;
    }
    else {
        double timeLimit = properties->getDouble("timeLimit");
        if (estimatedTimeToPoint < timeLimit) {
            // ROS_INFO_STREAM("can reach point within time");
            return Status::Success;
        }
        else {
            // ROS_INFO_STREAM("cannot reach point within time");
            return Status::Failure;
        }
    }
}
}
}