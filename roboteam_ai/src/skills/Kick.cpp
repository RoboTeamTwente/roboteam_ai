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

void Kick::Initialize() {
  amountOfCycles = 0;
}

bt::Node::Status Kick::Update() {

  // Gail if we did not succeed after a number of cycles
  amountOfCycles++;
  if (amountOfCycles > MAX_KICK_CYCLES) {
    return Status::Failure;
  }

  // Get kickVelocity from blackboard, otherwise it is a default value.
  double kickVel = blackboard->HasDouble("kickVel") ? blackboard->GetDouble("kickVel") : DEFAULT_KICK_POWER;

  // Send the robotCommand.
  roboteam_msgs::RobotCommand command;
  command.id = robot.id;
  // TODO check if we can avoid the casting to unsigned char without warnings
  command.dribbler = (unsigned char) true;
  command.kicker = (unsigned char) true;
  command.kicker_forced = (unsigned char) true;
  command.kicker_vel = (float) kickVel;

  io::robotCommandPublisher.publish(command);
  return Status::Running;
}

} // ai
} // rtt