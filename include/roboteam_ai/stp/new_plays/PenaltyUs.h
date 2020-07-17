//
// Created by timovdk on 5/1/20.
//

#ifndef RTT_PENALTYUS_H
#define RTT_PENALTYUS_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

class PenaltyUs : public Play {
 public:
  /**
   * Constructor that initializes roles with test roles
   */
  PenaltyUs();

  /**
   * Gets the score for the current play
   *
   * On the contrary to isValidPlay() this checks how good the play actually is
   * return in range of 0 - 100
   *
   * @param world World to get the score for (world_new::World::instance())
   * @return The score, 0 - 100
   */
  uint8_t score(world_new::World* world) noexcept override;

  /**
   * Assigns robots to roles of this play
   */
  Dealer::FlagMap decideRoleFlags() const noexcept override;

  void calculateInfoForRoles() noexcept override;

  bool shouldRoleSkipEndTactic() override;

  /**
   * Gets the play name
   */
  const char* getName() override;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_PENALTYUS_H
