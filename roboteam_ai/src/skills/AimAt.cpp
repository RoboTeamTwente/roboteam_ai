//
// Created by baris on 22/10/18.
//
namespace rtt {
namespace ai {
#include "AimAt.h"
rtt::ai::AimAt::AimAt(std::string name, bt::Blackboard::Ptr blackboard)
        :Skill(name, blackboard) {

}


bt::Node::Status rtt::ai::AimAt::Update(){

    roboteam_msgs::World world = LastWorld::get();
    bool setRosParam = blackboard->GetBool("setRosParam");

    int robotID = blackboard->GetInt("ROBOT_ID");
    // if (robotID == 1) ROS_INFO_STREAM("AimAt Update");
    std::string destination = blackboard->GetString("At");

    // Check is world contains a sensible message. Otherwise wait, it might the case that AimAt::Update
    // is called before the first world state update
    if (world.us.size() == 0) {
        return Status::Running;
    }

    Vector2 passTo;

    if (destination == "robot" || destination == "paramrobot"){
        int AtRobotID;
        if (destination == "robot") {
            AtRobotID = blackboard->GetInt("AtRobot");
        } else if (!ros::param::get(name + "_targetBot", AtRobotID)) {
            ROS_WARN("[AimAt] Mode is set to paramrobot, but the %s_targetBot parameter is not set!", name.c_str());
        }
        auto possibleBot = getWorldBot(AtRobotID);
        if (possibleBot) {
            passTo = Vector2(possibleBot->pos);
        } else {
            return Status::Failure;
        }
    } else if(destination == "theirgoal"){
        passTo = LastWorld::get_their_goal_center();
    } else if(destination == "ourgoal"){
        passTo = LastWorld::get_our_goal_center();
    } else if (destination == "position") {
        passTo = Vector2(blackboard->GetDouble("xGoal"), blackboard->GetDouble("yGoal"));
    } else if (destination == "paramrobot") {

    }

    private_bb->SetInt("ROBOT_ID", robotID);
    private_bb->SetString("center", "ball");
    private_bb->SetDouble("faceTowardsPosx", passTo.x);
    private_bb->SetDouble("faceTowardsPosy", passTo.y);
    double rotationSpeed = 2.0;
    if (blackboard->HasDouble("rotationSpeed")) {
        rotationSpeed = blackboard->GetDouble("rotationSpeed");
    }
    private_bb->SetDouble("w",rotationSpeed);
    private_bb->SetDouble("radius", 0.11);

    Status result = rotateAroundPoint.Update();
    if (result == Status::Success) {

        roboteam_msgs::RobotCommand command;
        command.id = robotID;
        command.kicker = blackboard->GetBool("passOn");
        command.kicker_forced = blackboard->GetBool("passOn");
        command.kicker_vel = blackboard->GetBool("passOn") ? 6.0 : 0;

        command.x_vel = 0;
        command.y_vel = 0;
        command.dribbler = !blackboard->GetBool("passOn");

        auto& pub = rtt::GlobalPublisher<roboteam_msgs::RobotCommand>::get_publisher();
        pub.publish(command);

        if (setRosParam) {
            set_PARAM_KICKING(true);
        }
        return Status::Success;
    } else if (result == Status::Failure) {
        // ROS_INFO_STREAM("AimAt failed :(");
        return Status::Failure;
    } else {
        return Status::Running;
    }
}










} // ai
} // rtt
