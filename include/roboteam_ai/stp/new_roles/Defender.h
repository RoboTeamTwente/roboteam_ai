//
// Created by jordi on 26-03-20.
//

#ifndef RTT_DEFENDER_H
#define RTT_DEFENDER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Defender : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    Defender(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_DEFENDER_H
