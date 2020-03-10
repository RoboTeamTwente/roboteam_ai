//
// Created by thijs on 6-7-19.
//

#ifndef ROBOTEAM_AI_BALLPLACEMENTFORMATION_H
#define ROBOTEAM_AI_BALLPLACEMENTFORMATION_H

#include <skills/formations/StopFormation.h>

namespace rtt::ai {
class BallPlacementFormation : public StopFormation {
   public:
    explicit BallPlacementFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    void updateFormation() override;
    Vector2 getFormationPosition() override;

    std::vector<world_new::view::RobotView> robotsInFormationPtr() override;
    static std::vector<world_new::view::RobotView> robotsInFormation;
    bool positionShouldBeAvoided(Vector2 pos) override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_BALLPLACEMENTFORMATION_H
