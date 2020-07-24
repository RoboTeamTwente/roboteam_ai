//
// Created by jordi on 11-05-20.
//

#ifndef RTT_BALLGETTER_H
#define RTT_BALLGETTER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class BallGetter : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    BallGetter(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_BALLGETTER_H
