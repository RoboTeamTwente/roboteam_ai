//
// Created by baris on 22/10/18.
//

#ifndef ROBOTEAM_AI_GOTOPOS_H
#define ROBOTEAM_AI_GOTOPOS_H

#include "Skill.h"
#include "roboteam_utils/Vector2.h"
#include  <boost/optional.hpp>
#include <roboteam_msgs/RobotCommand.h>

namespace rtt {
namespace ai {

class GoToPos : public Skill {
public:
    GoToPos(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
    void Initialize();

    void sendStopCommand(uint id);
    Vector2 getForceVectorFromRobot(Vector2 myPos, Vector2 otherRobotPos, Vector2 antenna, Vector2 sumOfForces);
    Vector2 avoidRobots(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces);
    Vector2 avoidDefenseAreas(Vector2 myPos, Vector2 myVel, Vector2 targetPos, Vector2 sumOfForces);
    Vector2 avoidBall(Vector2 ballPos, Vector2 myPos, Vector2 sumOfForces, Vector2 targetPos, Vector2 myVel);
    Vector2 checkTargetPos(Vector2 targetPos, Vector2 myPos);

    boost::optional<roboteam_msgs::RobotCommand> getVelCommand();

    Status Update();

//    static VerificationMap required_params() {
//        VerificationMap params;
//        params["ROBOT_ID"] = BBArgumentType::Int;
//        params["xGoal"] = BBArgumentType::Double;
//        params["yGoal"] = BBArgumentType::Double;
//        return params;
//    }
    std::string node_name() { return "GoToPos"; }

private:
    // output target: grsim or serial
    std::string robot_output_target;
    bool grsim;

    // Obstacle avoidance parameters
    double avoidRobotsGain;
    double cushionGain;
    double maxDistToAntenna;

    // Safety margins used to filter the target position
    double safetyMarginGoalAreas;
    double marginOutsideField;

    // Global blackboard info
    uint ROBOT_ID;
    uint KEEPER_ID;

    // Info about previous states
    Vector2 prevTargetPos;

    // Success
    int successCounter;
    bool succeeded;
//    bool failure;
//
//    Draw drawer;
//    Control controller;
//
//    // @DEBUG info
//    ros::Publisher myPosTopic;
//    ros::Publisher myVelTopic;
//    ros::Publisher myTargetPosTopic;
//
//    // a position a bit behind me that is determined in the initialize function
//    Vector2 backwardPos;
} ;

#endif //ROBOTEAM_AI_GOTOPOS_H
} // ai
} // rtt
