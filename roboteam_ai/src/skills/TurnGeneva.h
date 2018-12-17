//
// Created by kjhertenberg on 24-10-18.
//

#ifndef ROBOTEAM_AI_TURNGENEVA_H
#define ROBOTEAM_AI_TURNGENEVA_H

#include "Skill.h"

namespace rtt {
namespace ai {

class TurnGeneva : public Skill {
    private:
        int amountOfCycles;
    protected:
        void sendGenevaCommand(int genevaState);
    public:
        Status onUpdate() override;
        void onInitialize() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_TURNGENEVA_H
