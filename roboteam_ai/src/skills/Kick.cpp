//
// Created by mrlukasbos on 23-10-18.
//

#include <roboteam_msgs/RobotCommand.h>
#include <roboteam_msgs/WorldRobot.h>
#include <boost/optional.hpp>
#include "Kick.h"
#include "../utilities/World.h"
#include "../utilities/Constants.h"
#include "../io/RoleIOManager.h"

namespace rtt {
namespace ai {

bt::Node::Status Kick::Update() {

  int robotID = blackboard->GetInt("ROBOT_ID");
  boost::optional<roboteam_msgs::WorldRobot> robotPointer = World::getRobotForId(robotID, true);
  roboteam_msgs::WorldRobot robot;
  if (robotPointer) {
    robot = *robotPointer;
  } else {
    ROS_WARN("Kick: Robot not found");
    return Status::Failure;
  }

  double kickVel = blackboard->HasDouble("kickVel") ? blackboard->GetDouble("kickVel") : DEFAULT_KICK_POWER;

  roboteam_msgs::RobotCommand command;
  command.id = robotID;
  command.dribbler = true;
  command.kicker = true;
  command.kicker_forced = true;
  command.kicker_vel = kickVel;
  command.x_vel = 0.0;
  command.y_vel = 0.0;
  command.w = 0.0;

  io::robotCommandPublisher.publish(command);
  return Status::Running;
}

} // ai
} // rtt