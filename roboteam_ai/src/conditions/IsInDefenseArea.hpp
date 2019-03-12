//
// Created by rolf on 17-10-18.
//

#ifndef ROBOTEAM_AI_ISINDEFENSEAREA_HPP
#define ROBOTEAM_AI_ISINDEFENSEAREA_HPP

#include "Condition.h"

namespace rtt {
namespace ai {

class IsInDefenseArea : public ai::Condition {
    private:
        using status = bt::Node::Status;
        bool ourDefenseArea;
        bool outsideField;
        float margin;
    public:
        explicit IsInDefenseArea(std::string name = "", bt::Blackboard::Ptr blackboard = nullptr);
        Status update() override;
        std::string node_name() override { return "IsInDefenseArea"; }
};

}// ai
}// rtt

#endif //ROBOTEAM_AI_ISINDEFENSEAREA_HPP
