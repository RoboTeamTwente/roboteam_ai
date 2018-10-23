//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_SHOOT_H
#define ROBOTEAM_AI_SHOOT_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Kick : public Skill {
 private:
  int amountOfCycles;
 public:
  Status Update() override;
  void Initialize() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SHOOT_H
