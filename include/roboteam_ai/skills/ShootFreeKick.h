//
// Created by baris on 14-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTFREEKICK_H
#define ROBOTEAM_AI_SHOOTFREEKICK_H

#include "Skill.h"

namespace rtt::ai {

class ShootFreeKick : public Skill {
   public:
    explicit ShootFreeKick(std::string name, bt::Blackboard::Ptr blackboard);
    Status onUpdate() override;
    void onInitialize() override;
    void onTerminate(Status s) override;

   private:
    enum Progress { GOING, TARGETING, READY, SHOOTING };
    int counter = 0;

    Progress progress;
    Vector2 targetPos;
    double errorMarginPos = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03;  // Same logic
    bool isShot();
    Vector2 freeKickPos;
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_SHOOTFREEKICK_H
