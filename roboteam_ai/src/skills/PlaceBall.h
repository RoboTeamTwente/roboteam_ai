//
// Created by mrlukasbos on 22-10-18.
//

#ifndef ROBOTEAM_AI_PLACEBALL_H
#define ROBOTEAM_AI_PLACEBALL_H

#include "Skill.h"

namespace rtt {
namespace ai {

class PlaceBall : public Skill {
 private:

 public:
  Status Update() override;

};

} // ai
} // rtt

#endif //ROBOTEAM_AI_PLACEBALL_H
