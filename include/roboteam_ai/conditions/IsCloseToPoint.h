#ifndef ROBOTEAM_AI_ISCLOSETOPOINT_H
#define ROBOTEAM_AI_ISCLOSETOPOINT_H

#include "Condition.h"

namespace rtt::ai {

    class IsCloseToPoint : public Condition {
        private:
        double margin = 0.0;
        Vector2 position;

        public:
        explicit IsCloseToPoint(std::string name = "IsCloseToPoint", bt::Blackboard::Ptr blackboard = nullptr);

        void onInitialize() override;

        Status onUpdate() override;
    };

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_ISCLOSETOPOINT_H
