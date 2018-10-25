//
// Created by mrlukasbos on 23-10-18.
//

#include "Kick.h"

namespace rtt {
namespace ai {

Kick::Kick(std::string name, bt::Blackboard::Ptr blackboard) : Skill(name, blackboard) { }

  void Kick::Initialize() {
  amountOfCycles = 0;
}

bt::Node::Status Kick::Update() {
  // Fail if we did not succeed after a number of cycles
  amountOfCycles++;
  if (amountOfCycles > MAX_KICK_CYCLES) {
    return Status::Failure;
  }

  // Get kickVelocity from blackboard, otherwise it is a default value.
  double kickVel = blackboard->HasDouble("kickVel") ? blackboard->GetDouble("kickVel") : DEFAULT_KICK_POWER;

  // Send the robotCommand.
  sendKickCommand(kickVel);

  return Status::Running;
}

void Kick::sendKickCommand(double kickVel) {
  roboteam_msgs::RobotCommand command;
  command.id = robot.id;
  // TODO check if we can avoid the casting to unsigned char without warnings
  command.kicker = (unsigned char) true;
  command.kicker_forced = (unsigned char) true;
  command.kicker_vel = (float) kickVel;

  publishRobotCommand(command);
}


} // ai
} // rtt0