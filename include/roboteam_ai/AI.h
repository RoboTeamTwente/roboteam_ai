//
// Created by rolf on 15-02-21.
//

#ifndef RTT_ROBOTEAM_AI_SRC_AI_H_
#define RTT_ROBOTEAM_AI_SRC_AI_H_

#include "stp/PlayChecker.hpp"
#include "stp/PlayDecider.hpp"

namespace rtt {
class AI {
 public:
  AI();

  void decidePlay(world::World *_world);

 private:
  /**
   * Current best play as picked by checker + decider
   */
  rtt::ai::stp::Play *currentPlay{nullptr};

  /**
   * Checks which plays are valid out of all the plays
   */
  rtt::ai::stp::PlayChecker playChecker;
  /**
   * Checks, out of the valid plays, which play is the best to choose
   */
  rtt::ai::stp::PlayDecider playDecider;
  /**
   * Function that decides whether to change plays given a world and field.
   * @param _world the current world state
   * @param field the current field state
   */

  /**
  * The vector that contains all plays
  */
  std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;
};
}
#endif //RTT_ROBOTEAM_AI_SRC_AI_H_
