//
// Created by jordi on 26-03-20.
//

#ifndef RTT_BALLDEFENDER_H
#define RTT_BALLDEFENDER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class BallDefender : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    BallDefender(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_BALLDEFENDER_H
