//
// Created by mrlukasbos on 23-10-18.
//

#ifndef ROBOTEAM_AI_SHOOT_H
#define ROBOTEAM_AI_SHOOT_H

#include "Skill.h"

namespace rtt {
namespace ai {

class Kick : public Skill {
    private:
        int amountOfCycles{};
    protected:
        virtual void sendKickCommand(double kickVel);
        enum Progression {
          KICKING, DONE, FAIL
        };
        Progression currentProgress;

    public:
        explicit Kick(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        Status onUpdate() override;
        void onInitialize() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_SHOOT_H
