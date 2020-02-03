//
// Created by mrlukasbos on 26-4-19.
//

#ifndef ROBOTEAM_AI_CANPLAY_H
#define ROBOTEAM_AI_CANPLAY_H

#include "Condition.h"

namespace rtt::ai {

    class CanPlay : public Condition {
        public:
        explicit CanPlay(std::string name = "CanPlay", bt::Blackboard::Ptr blackboard = nullptr);

        Status onUpdate() override;
    };

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_CANPLAY_H
