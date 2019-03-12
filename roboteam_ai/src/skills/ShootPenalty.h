//
// Created by baris on 11-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTPENALTY_H
#define ROBOTEAM_AI_SHOOTPENALTY_H

#include "Skill.h"
namespace rtt {
namespace ai {

class ShootPenalty : public Skill {
    public:
        explicit ShootPenalty(string name, bt::Blackboard::Ptr blackboard);

    private:
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;
        Angle geneva = 0.349066; // 20 degrees
        Angle fakeOffset = 0.0872665; // 5 degrees
        Vector2 targetPos;
        enum Progress {
          GOING,
          ROTATING,
          READY,
          SHOOTING,
          AVOIDING
        };
        Progress progress;
        double errorMarginPos = Constants::PENALTY_SHOOT_MARGIN() + 0.02;
        double errorMarginAng = 0.0174533; // 1 degrees

        control::PositionController goToPos;


};

}
}

#endif //ROBOTEAM_AI_SHOOTPENALTY_H
