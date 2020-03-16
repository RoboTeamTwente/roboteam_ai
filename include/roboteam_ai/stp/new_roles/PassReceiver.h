//
// Created by jessevw on 16.03.20.
//

#ifndef RTT_PASSRECEIVE_H
#define RTT_PASSRECEIVE_H

#include "stp/Role.hpp"

namespace rtt::ai::stp::role {
    class PassReceiver : Role {
    public:
        /**
         * Ctor that sets the name of the role and creates a statemachine of tactics
         * @param name name of the role
         */
        PassReceiver(std::string name);
    };
}

#endif //RTT_PASSRECEIVE_H
