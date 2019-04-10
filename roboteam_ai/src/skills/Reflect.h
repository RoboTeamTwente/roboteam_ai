//
// Created by robzelluf on 4/9/19.
//

#ifndef ROBOTEAM_AI_REFLECT_H
#define ROBOTEAM_AI_REFLECT_H

#include "Skill.h"
#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>

namespace rtt {
namespace ai {

class Reflect : public Skill {
public:
    explicit Reflect(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

}
}


#endif //ROBOTEAM_AI_REFLECT_H
