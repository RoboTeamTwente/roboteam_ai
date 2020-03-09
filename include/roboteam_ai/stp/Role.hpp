//
// Created by john on 3/9/20.
//

#ifndef RTT_ROLE_HPP
#define RTT_ROLE_HPP
#include <vector>

#include "Tactic.h"

namespace rtt::ai::stp {

    class Role {
    public:
        

    private:
        uint8_t robotId;
        std::vector<std::unique_ptr<Tactic>> tacticsForRobot;
    };

} // namespace rtt::ai::analysis

#endif //RTT_ROLE_HPP
