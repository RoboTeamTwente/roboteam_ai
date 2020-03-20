//
// Created by rolf on 10-4-19.
//

#ifndef ROBOTEAM_AI_INTERCEPTROBOT_HPP
#define ROBOTEAM_AI_INTERCEPTROBOT_HPP

#include "Skill.h"

namespace rtt::ai {
class InterceptRobot : public Skill {
   private:
    Vector2 getInterceptPos(world_new::view::RobotView robotToIntercept);

   public:
    explicit InterceptRobot(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_INTERCEPTROBOT_HPP
