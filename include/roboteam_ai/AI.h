//
// Created by rolf on 15-02-21.
//

#ifndef RTT_ROBOTEAM_AI_SRC_AI_H_
#define RTT_ROBOTEAM_AI_SRC_AI_H_

#include "stp/PlayChecker.hpp"
#include "stp/PlayDecider.hpp"
#include "AISettings.h"
#include "world/World.hpp"
#include <roboteam_proto/State.pb.h>
#include <roboteam_proto/AICommand.pb.h>
#include "control/ControlModule.h"

namespace rtt {
class AI {
 public:
  explicit AI(int id);

  void updateState(const proto::State& state);
  proto::AICommand decidePlay();
  [[nodiscard]] proto::Handshake getButtonDeclarations() const;
  [[nodiscard]] proto::Handshake getSettingValues() const;
  void updateSettings(const proto::UiValues& values);
 private:
  //updates referee-related information to the game state manager, and makes sure things such as rotation, color etc. are correct
  void updateSettingsReferee(const proto::State& state);

  void onSideOrColorChanged();
  std::unique_ptr<world::World> world = nullptr;
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

  rtt::ai::stp::PlayEvaluator playEvaluator;

  /**
   * The vector that contains all plays
   */
  std::vector<std::unique_ptr<rtt::ai::stp::Play>> plays;

  AISettings settings;

  //std::unique_ptr<ai::control::ControlModule> control_module; TODO: fix staticness
};
}
#endif //RTT_ROBOTEAM_AI_SRC_AI_H_
