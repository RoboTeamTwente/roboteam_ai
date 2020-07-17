//
// Created by jordi on 27-03-20.
//

#ifndef RTT_DEFENDSHOT_H
#define RTT_DEFENDSHOT_H

#include "stp/Play.hpp"

namespace rtt::ai::stp::play {

/**
 * DefendShot Play is executed when the opponent has or is close to the ball and on our side of the field.
 * In this case they most likely will try to score. Some defenders defend the goal by blocking the path between enemy
 * robots and the goal. Other defenders block other enemy robots to avoid passes to them.
 */
class DefendShot : public Play {
 public:
  /**
   * Constructor that initializes roles with roles that are necessary for this play
   */
  DefendShot();

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

  /**
   * Calculates info for the roles
   */
  void calculateInfoForRoles() noexcept override;

  /**
   * Gets the play name
   */
  const char* getName() override;

 protected:
  bool shouldRoleSkipEndTactic() override;

  /**
   * Calculates info for the defenders
   */
  void calculateInfoForDefenders() noexcept;

  /**
   * Calculates info for the harassers
   */
  void calculateInfoForHarassers() noexcept;

  /**
   * Calculates info for the keeper
   */
  void calculateInfoForKeeper() noexcept;

  /**
   * Calculates info for the midfielders
   */
  void calculateInfoForMidfielders() noexcept;

  /**
   * Calculates info for the offenders
   */
  void calculateInfoForOffenders() noexcept;
};
}  // namespace rtt::ai::stp::play

#endif  // RTT_DEFENDSHOT_H
