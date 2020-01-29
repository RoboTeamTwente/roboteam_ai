//
// Created by thijs on 6-7-19.
//
#include <skills/formations/StopFormation.h>

#ifndef ROBOTEAM_AI_BALLPLACEMENTFORMATION_H
#define ROBOTEAM_AI_BALLPLACEMENTFORMATION_H

namespace rtt::ai {
class BallPlacementFormation : public StopFormation {
 public:
  explicit BallPlacementFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

 private:
  void updateFormation() override;
  Vector2 getFormationPosition() override;

  std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormationPtr() override;
  static std::shared_ptr<std::vector<std::shared_ptr<Robot>>> robotsInFormation;
  bool positionShouldBeAvoided(Vector2 pos) override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLPLACEMENTFORMATION_H
