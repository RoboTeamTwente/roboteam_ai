#ifndef ROBOTEAM_AI_IHAVEBALL_HPP
#define ROBOTEAM_AI_IHAVEBALL_HPP
/**
 * Why random .hpp?...
 * Either stick to .h or .hpp for consistency purposes
 */
#include "Condition.h"

namespace rtt {
namespace ai {

class HasBall : public Condition {
public:
    explicit HasBall(std::string name = "HasBall", bt::Blackboard::Ptr blackboard = nullptr);
    Status onUpdate() override;
};

} // ai
} // rtt

#endif //ROBOTEAM_AI_IHAVEBALL_HPP
