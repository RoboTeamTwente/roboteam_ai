//
// Created by rolf on 04/12/18.
//

#ifndef ROBOTEAM_AI_GETBALL_H
#define ROBOTEAM_AI_GETBALL_H

#include "Skill.h"

namespace rtt {
namespace ai {

class GetBall : public Skill {

        const double COLLISION_RADIUS = 0.18;
        const double ANGLE_SENS = 0.05*M_PI;
        const double MAX_RANGE = 0.7;
        const int POSSES_BALL_CYCLES = 25;
        const double SPEED = (Constants::GRSIM() ? 0.4 : 0.8);
        const double OVERSHOOT = .02;

    private:
        enum Progression {
          TURNING, APPROACHING, OVERSHOOTING, DRIBBLING, SUCCESS, FAIL
        };

        Progression currentProgress;
        void checkProgression();

        void sendTurnCommand();
        void sendApproachCommand();
        void sendDribblingCommand();
        void sendOvershootCommand();

        bool botHasLastVisibleBall();
        int count;
        Vector2 deltaPos;
        Vector2 lastVisibleBallPos;
        Vector2 approachPos;
        bool lockAngle;
        double lockedAngle;
        int currentTick, maxTicks;
    public:
        explicit GetBall(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;

};
}
}

#endif //ROBOTEAM_AI_GETBALL_H
