//
// Created by rolf on 10/12/18.
//

#ifndef ROBOTEAM_AI_KEEPER_H
#define ROBOTEAM_AI_KEEPER_H

#include "Skill.h"

namespace rtt::ai {
class Keeper : public Skill {
    const double MIN_ATTACKER_DIST = 0.3;

   private:
    Arc blockCircle;
    Vector2 computeBlockPoint(const Vector2 &defendPos);
    Vector2 goalPos;
    double goalWidth;
    void setGoalPosWithAttacker(world_new::view::RobotView attacker);
    rtt::Arc createKeeperArc();

   public:
    explicit Keeper(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
    void onTerminate(Status s) override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_KEEPER_H
