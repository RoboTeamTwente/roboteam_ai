//
// Created by rolf on 15-02-21.
//

#ifndef RTT_ROBOTEAM_AI_SRC_AI_H_
#define RTT_ROBOTEAM_AI_SRC_AI_H_

#include "stp/PlayChecker.hpp"
#include "stp/PlayDecider.hpp"
#include "AISettings.h"

namespace rtt {
class AI {
 public:
  explicit AI(int id);

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
   * The vector that contains all plays
   */
  std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;

  AISettings settings;
};
}
#endif //RTT_ROBOTEAM_AI_SRC_AI_H_
