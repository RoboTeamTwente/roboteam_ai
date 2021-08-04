//
// Created by RobotJesse
//

#ifndef RTT_BALLPLACER_H
#define RTT_BALLPLACER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

class BallPlacer : public Role {
   public:
    /**
     * Ctor that sets the name of the role and creates a state machine of tactics
     * @param name name of the role
     */
    BallPlacer(std::string name);
};
}  // namespace rtt::ai::stp::role

#endif  // RTT_BALLPLACER_H
