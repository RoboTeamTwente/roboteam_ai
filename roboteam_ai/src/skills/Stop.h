//
// Created by baris on 3-4-19.
//

#ifndef ROBOTEAM_AI_STOP_H
#define ROBOTEAM_AI_STOP_H

#include "Skill.h"
namespace rtt {
namespace ai {

class Stop : public Skill {
    public:
        explicit Stop(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
        void onTerminate(Status s) override;
    private:
        enum active : short {
          DEFENSIVE,
          OFFENSIVE
        };
};
}}

#endif //ROBOTEAM_AI_STOP_H
