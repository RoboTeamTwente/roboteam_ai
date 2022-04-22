//
// Created Alexander on 22-04-2022
//

#ifndef RTT_FREEKICKTAKER_H
#define RTT_FREEKICKTAKER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class FreeKickTaker : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    FreeKickTaker(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_FREEKICKTAKER_H
