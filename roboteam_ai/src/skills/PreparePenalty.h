//
// Created by baris on 27-2-19.
//

#ifndef ROBOTEAM_AI_PREPAREPENALTY_H
#define ROBOTEAM_AI_PREPAREPENALTY_H

#include "Skill.h"

namespace rtt {
namespace ai {

class PreparePenalty : public Skill {

        /**
         * Things to prepare for a penalty
         * 1. Keeper calculation
         * 2. Geneva
         * 3. Orientation
         * 4. Then wait
         */
    public:
        explicit PreparePenalty(string name, bt::Blackboard::Ptr blackboard);
        void onInitialize() override;
        Status onUpdate() override;
};
}
}

#endif //ROBOTEAM_AI_PREPAREPENALTY_H
