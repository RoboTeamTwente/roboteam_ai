//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_CHIP_H
#define ROBOTEAM_AI_CHIP_H

#include "Kick.h"
#include "roboteam_msgs/RobotCommand.h"

namespace rtt {
namespace ai {

class Chip : public Kick {
 public:
  void sendKickCommand(double kickVel) override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CHIP_H
