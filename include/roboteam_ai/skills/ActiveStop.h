//
// Created by baris on 8-4-19.
//

#ifndef ROBOTEAM_AI_ACTIVESTOP_H
#define ROBOTEAM_AI_ACTIVESTOP_H
#include <control/numtrees/NumTreePosControl.h>
#include "Skill.h"

namespace rtt::ai {

class ActiveStop : public Skill {
   public:
    explicit ActiveStop(std::string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;

   private:
    Vector2 targetPos;
    static int attack;
    bool attacker = false;
    static Vector2 getOffensiveActivePoint(const Field &field);
    static Vector2 getDefensiveActivePoint(const Field &field);
    static Vector2 getPoint(const Field &field, const Vector2 &penaltyPos);
};
}  // namespace rtt::ai
#endif  // ROBOTEAM_AI_ACTIVESTOP_H