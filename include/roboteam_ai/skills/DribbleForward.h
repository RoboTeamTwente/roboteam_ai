//
// Created by robzelluf on 5/13/19.
//

#ifndef ROBOTEAM_AI_DRIBBLEFORWARD_H
#define ROBOTEAM_AI_DRIBBLEFORWARD_H

#include <control/BasicPosControl.h>
#include <control/ballHandling/BallHandlePosControl.h>
#include "Skill.h"

namespace rtt::ai {

class DribbleForward : public Skill {
 private:
  Vector2 initialBallPos;
  double dribbleDistance = 0.8;
  Vector2 targetPos;
  control::BasicPosControl basicGtp;
  control::BallHandlePosControl ballHandlePosControl;

 public:
  explicit DribbleForward(std::string name, bt::Blackboard::Ptr blackboard);
  void onInitialize() override;
  Status onUpdate() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_DRIBBLEFORWARD_H
