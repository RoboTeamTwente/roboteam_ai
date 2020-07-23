//
// Created by jordi on 19-05-20.
//

#ifndef RTT_BALLREFLECTOR_H
#define RTT_BALLREFLECTOR_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class BallReflector : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    BallReflector(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_BALLREFLECTOR_H
