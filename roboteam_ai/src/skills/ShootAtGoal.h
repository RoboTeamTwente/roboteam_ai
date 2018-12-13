//
// Created by baris on 13-12-18.
//
#ifndef ROBOTEAM_AI_SHOOTATGOAL_H
#define ROBOTEAM_AI_SHOOTATGOAL_H


#include "Skill.h"


namespace rtt {
namespace ai {

class ShootAtGoal : public Skill {

    public:
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;
    private:
        bool onlyGeneva = false;
        bool neverGeneva = false;

        enum Progression {
          READY, DONE, ORIENTATE, TURN_GENEVA,
        };


};
}
}

#endif //ROBOTEAM_AI_SHOOTATGOAL_H
