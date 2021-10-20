//
// Created by jesse on 30-04-20.
//

#ifndef RTT_BALLAVOIDER_H
#define RTT_BALLAVOIDER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class BallAvoider : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    BallAvoider(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_BALLAVOIDER_H
