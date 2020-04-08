//
// Created by jordi on 08-04-20.
//

#ifndef RTT_KEEPER_H
#define RTT_KEEPER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class Keeper : public Role {
public:
    /**
     * Ctor that sets the name of the role and creates a statemachine of tactics
     * @param name name of the role
     */
    Keeper(std::string name);
};

} // namespace rtt::ai::stp::role

#endif //RTT_KEEPER_H
