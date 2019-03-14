//
// Created by baris on 14-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTFREEKICK_H
#define ROBOTEAM_AI_SHOOTFREEKICK_H

#include "Skill.h"

namespace rtt {
namespace ai {

class ShootFreeKick : public Skill {

    public:
        explicit ShootFreeKick(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;


    private:
        enum Progress {
          GOING,
          TARGETING,
          READY,
          SHOOTING
        };

        Progress progress;
        Vector2 targetPos;
        control::PositionController goToPos;
        double errorMarginPos = Constants::PENALTY_SHOOT_MARGIN() + 0.02; // Same logic

};

}
}

#endif //ROBOTEAM_AI_SHOOTFREEKICK_H
