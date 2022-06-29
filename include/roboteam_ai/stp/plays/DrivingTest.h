//
// Created by agata on 29/06/2022.
//

#ifndef RTT_DrivingTest_H
#define RTT_DrivingTest_H

#include "stp/Play.hpp"

namespace rtt::ai::stp {

class DrivingTest : public Play {
   public:
    /**
     * Constructor that initializes roles with test roles
     */
    DrivingTest();

    /**
     * Calculates the score of this play to determine which play is best in this situation
     * @param field The current Field
     * @return The score of this play (0-255)
     */
    uint8_t score(const rtt::world::Field& field) noexcept override;

    /**
     * Assigns robots to roles of this play
     */
    Dealer::FlagMap decideRoleFlags() const noexcept override;

    /**
     * Calculates info for the roles
     */
    void calculateInfoForRoles() noexcept override;

    /**
     * Calculate info for the roles that need to be calculated for scoring
     */
    void calculateInfoForScoredRoles(world::World*) noexcept override{};

    /**
     * Gets the play name
     */
    const char* getName() override;
};
}  // namespace rtt::ai::stp

#endif  // RTT_DrivingTest_H
