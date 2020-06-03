//
// Created by jordi on 19-05-20.
//

#ifndef RTT_BALLREFLECTER_H
#define RTT_BALLREFLECTER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class BallReflecter : public Role {
public:
    /**
     * Ctor that sets the name of the role and creates a statemachine of tactics
     * @param name name of the role
     */
    BallReflecter(std::string name);
};

} // namespace rtt::ai::stp::role

#endif // RTT_BALLREFLECTER_H
