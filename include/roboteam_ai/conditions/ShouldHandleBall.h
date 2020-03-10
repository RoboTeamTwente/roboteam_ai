//
// Created by rolf on 12-4-19.
//

#ifndef ROBOTEAM_AI_SHOULDHANDLEBALL_H
#define ROBOTEAM_AI_SHOULDHANDLEBALL_H

#include "Condition.h"

namespace rtt::ai {

class ShouldHandleBall : public Condition {
   public:
    explicit ShouldHandleBall(std::string name = "ShouldHandleBall", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
    void onTerminate(Status s) override;
    std::string node_name() override;
};
}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_SHOULDHANDLEBALL_H
