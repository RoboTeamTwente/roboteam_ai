//
// Created by jordi on 08-04-20.
//

#ifndef RTT_KEEPER_H
#define RTT_KEEPER_H

#include "stp/Role.hpp"
#include "world/Field.h"

namespace rtt::ai::stp::role {

class Keeper : public Role {
public:
    /**
     * Ctor that sets the name of the role and creates a statemachine of tactics
     * @param name name of the role
     */
    Keeper(std::string name);

    [[nodiscard]] Status update(StpInfo const &info) noexcept override;

private:
    [[nodiscard]] bool isBallInOurDefenseAreaAndStill(world::Field field, Vector2 ballPos, Vector2 ballVel) noexcept;
};

} // namespace rtt::ai::stp::role

#endif //RTT_KEEPER_H
