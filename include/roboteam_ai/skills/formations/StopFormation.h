//
// Created by roboteam on 4/07/19.
//

#ifndef ROBOTEAM_AI_STOPFORMATION_H
#define ROBOTEAM_AI_STOPFORMATION_H

#include "Formation.h"

namespace rtt::ai {

class StopFormation : public Formation {
   public:
    explicit StopFormation(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   protected:
    Vector2 getFormationPosition() override;
    void updateFormation() override;
    void setFinalAngle() override;
    std::vector<std::vector<Vector2>> getStopPositions();
    virtual bool positionShouldBeAvoided(Vector2 pos);
    std::vector<Vector2> getProperPositions(int amount);

   private:
    std::vector<world_new::view::RobotView> robotsInFormationPtr() override;
    static std::vector<world_new::view::RobotView> robotsInFormation;
};

}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_STOPFORMATION_H
