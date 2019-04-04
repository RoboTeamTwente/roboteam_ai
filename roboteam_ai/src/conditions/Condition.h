#ifndef ROBOTEAM_AI_CONDITION_H
#define ROBOTEAM_AI_CONDITION_H

#include "../bt/Leaf.hpp"


namespace rtt {
namespace ai {

//forward declare control utils
namespace control {
    class ControlUtils;
}

class Condition : public bt::Leaf {
    protected:
        using Control = control::ControlUtils;
        using Status = bt::Node::Status;

    public:
        explicit Condition(std::string name, bt::Blackboard::Ptr blackboard = nullptr);

        std::string node_name() override;
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;

        virtual void onInitialize() { };
        virtual Status onUpdate() = 0;
        virtual void onTerminate(Status s) { };
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_CONDITION_H
