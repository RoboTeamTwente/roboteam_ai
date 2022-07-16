//
// Created by robopc on 15-7-22.
//

#ifndef RTT_PENALTYTAKER_H
#define RTT_PENALTYTAKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class PenaltyTaker : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    PenaltyTaker(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_PENALTYTAKER_H
