//
// Created by thijs on 19-11-18.
//

#include "GoToPos.h"

#ifndef ROBOTEAM_AI_GOTOPOSLUTH_OLD_H
#define ROBOTEAM_AI_GOTOPOSLUTH_OLD_H

namespace rtt::ai {
class SkillGoToPos : public GoToPos {
 private:
  bool goToBall;

  enum Progression { ON_THE_WAY, DONE, FAIL };
  Progression currentProgress;
  Progression checkProgression();

 public:
  explicit SkillGoToPos(std::string name, bt::Blackboard::Ptr blackboard);
  void gtpInitialize() override;
  Status gtpUpdate() override;
  void gtpTerminate(Status s) override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_GOTOPOSLUTH_H
