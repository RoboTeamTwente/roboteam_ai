//
// Created by baris on 25-4-19.
//

#ifndef ROBOTEAM_AI_DEFENDFREEKICK_H
#define ROBOTEAM_AI_DEFENDFREEKICK_H

#include "Formation.h"

namespace rtt::ai {

class DefendFreeKick : public Formation {
   public:
    explicit DefendFreeKick(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

   private:
    Vector2 getFormationPosition() override;
    std::vector<world_new::view::RobotView> robotsInFormationPtr() override;
    static std::vector<world_new::view::RobotView> robotsInFormation;
    static std::vector<Vector2> posses;
    void onTerminate(Status s) override;
};
}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_DEFENDFREEKICK_H
