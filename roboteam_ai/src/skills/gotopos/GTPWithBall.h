//
// Created by thijs on 26-4-19.
//

#ifndef ROBOTEAM_AI_GTPWITHBALL_H
#define ROBOTEAM_AI_GTPWITHBALL_H

#include "GoToPos.h"

namespace rtt {
namespace ai {

class GTPWithBall : public GoToPos {
    private:

    public:
        explicit GTPWithBall(string name, bt::Blackboard::Ptr blackboard);

        void gtpInitialize() override;
        Status gtpUpdate() override;
        void gtpTerminate(Status s) override;
};

}
}

#endif //ROBOTEAM_AI_GTPWITHBALL_H
