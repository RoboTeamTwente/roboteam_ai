//
// Created by Emiel on 23-01-22.
// This play allows for a robot to be selected via the interface and sent to a specific location on the field
//

#ifndef RTT_CONTROLSELECTEDROBOT_H
#define RTT_CONTROLSELECTEDROBOT_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {
class ControlSelectedRobot : public Play {
   public:
    ControlSelectedRobot();

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

    /**
     * Check if play should end
     */
    bool shouldEndPlay() noexcept override;

   private:
    // Id of the robot currently selected in the interface. -1 if no robot is selected
    int current_robot_id = -1;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_CONTROLSELECTEDROBOT_H
