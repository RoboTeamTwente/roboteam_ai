//
// Created by baris on 12-12-18.
//

#ifndef ROBOTEAM_AI_HARASS_H
#define ROBOTEAM_AI_HARASS_H

#include "Skill.h"


namespace rtt {
namespace ai {

class Harass : public Skill {

    public:
       void initialize() override;
       Status update() override;


    private:
       int harassmentTarget = -1;
       void pickHarassmentTarget();
       bool harassBallOwner = false;



};

}
}
#endif //ROBOTEAM_AI_HARASS_H
