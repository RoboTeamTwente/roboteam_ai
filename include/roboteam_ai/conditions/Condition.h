#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include <world_new/FieldComputations.hpp>
#include "treeinterp/Leaf.h"

namespace rtt::ai {

class Condition : public bt::Leaf {
   protected:
    using Status = bt::Node::Status;

   public:
    explicit Condition(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

    std::string node_name() override;
    /** Sets the ball. Sets the correct robot based on the blackboard. Calls onInitialize() */
    void initialize() override;
    Status update() override;
    void terminate(Status s) override;

    virtual void onInitialize(){};
    virtual Status onUpdate() = 0;
    virtual void onTerminate(Status s){};
};

}  // namespace rtt::ai

#endif  // ROBOTEAM_AI_CONDITION_H
