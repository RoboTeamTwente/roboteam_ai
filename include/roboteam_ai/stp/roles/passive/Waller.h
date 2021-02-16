//
// Created by maxl on 15-02-21.
//

#ifndef RTT_WALLER_H
#define RTT_WALLER_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {

    class Waller : public Role {
    public:
        /**
         * Ctor that sets the name of the role and creates a state machine of tactics
         * @param name name of the role
         */
        Waller(std::string name);
    };
}  // namespace rtt::ai::stp::role

#endif //RTT_WALLER_H
