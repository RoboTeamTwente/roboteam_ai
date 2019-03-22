//
// Created by baris on 11-3-19.
//

#ifndef ROBOTEAM_AI_SHOOTPENALTY_H
#define ROBOTEAM_AI_SHOOTPENALTY_H

#include "Skill.h"
#include "../utilities/Field.h"
namespace rtt {
namespace ai {

class ShootPenalty : public Skill {
    public:
        explicit ShootPenalty(string name, bt::Blackboard::Ptr blackboard);
        Status onUpdate() override;
        void onInitialize() override;
        void onTerminate(Status s) override;



    private:

        // TODO : make some more faking logic
        Angle geneva = 0.349066; // 20 degrees
        Angle fakeOffset = 0.0872665; // 5 degrees

        Vector2 targetPos;
        enum Progress {
          GOING,
          ROTATING,
          READY,
          SHOOTING
        };
        Progress progress;
        double errorMarginPos = Constants::ROBOT_RADIUS() + Constants::BALL_RADIUS() + 0.03;
        double errorMarginAng = 0.0174533; // 1 degrees
        bool isPenaltyShot();
        int count = 0;



        control::PositionController goToPos;


};

}
}

#endif //ROBOTEAM_AI_SHOOTPENALTY_H
