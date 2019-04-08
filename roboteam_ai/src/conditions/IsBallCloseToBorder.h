//
// Created by robzelluf on 2/18/19.
//

#ifndef ROBOTEAM_AI_ISBALLCLOSETOBORDER_H
#define ROBOTEAM_AI_ISBALLCLOSETOBORDER_H

#include "Condition.h"
#include <roboteam_ai/src/utilities/Constants.h>
#include "roboteam_ai/src/world/Field.h"

namespace rtt {
namespace ai {

class IsBallCloseToBorder : public Condition {
private:
    double margin;
    bool layingStill = false;
public:
    explicit IsBallCloseToBorder(std::string name = "IsBallCloseToBorder", bt::Blackboard::Ptr blackboard = nullptr);
    void onInitialize() override;
    Status onUpdate() override;
    std::string node_name() override;
};

}
}

#endif //ROBOTEAM_AI_ISBALLCLOSETOBORDER_H
