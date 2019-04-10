//
// Created by robzelluf on 4/9/19.
//

#ifndef ROBOTEAM_AI_REFLECTKICK_H
#define ROBOTEAM_AI_REFLECTKICK_H

#include "Skill.h"
#include <roboteam_ai/src/coach/PassCoach.h>
#include <roboteam_ai/src/coach/BallplacementCoach.h>

namespace rtt {
namespace ai {

class ReflectKick : public Skill {
public:
    explicit ReflectKick(string name, bt::Blackboard::Ptr blackboard);
    void onInitialize() override;
    Status onUpdate() override;
    void onTerminate(Status s) override;
};

}
}


#endif //ROBOTEAM_AI_REFLECTKICK_H
