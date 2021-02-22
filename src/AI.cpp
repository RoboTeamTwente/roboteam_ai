//
// Created by rolf on 15-02-21.
//

#include "AI.h"

/**
 * Plays are included here
 */
#include "stp/plays/AggressiveStopFormation.h"
#include "stp/plays/Attack.h"
#include "stp/plays/AttackingPass.h"
#include "stp/plays/BallPlacementThem.h"
#include "stp/plays/BallPlacementUs.h"
#include "stp/plays/DefendPass.h"
#include "stp/plays/DefendShot.h"
#include "stp/plays/DefensiveStopFormation.h"
#include "stp/plays/FreeKickThem.h"
#include "stp/plays/GenericPass.h"
#include "stp/plays/GetBallPossession.h"
#include "stp/plays/GetBallRisky.h"
#include "stp/plays/Halt.h"
#include "stp/plays/KickOffThem.h"
#include "stp/plays/KickOffThemPrepare.h"
#include "stp/plays/KickOffUs.h"
#include "stp/plays/KickOffUsPrepare.h"
#include "stp/plays/PenaltyThem.h"
#include "stp/plays/PenaltyThemPrepare.h"
#include "stp/plays/PenaltyUs.h"
#include "stp/plays/PenaltyUsPrepare.h"
#include "stp/plays/ReflectKick.h"
#include "stp/plays/TestPlay.h"
#include "stp/plays/TimeOut.h"

rtt::AI::AI() {
  plays = std::vector<std::unique_ptr<rtt::ai::stp::Play>>{};

  /// This play is only used for testing purposes, when needed uncomment this play!
  // plays.emplace_back(std::make_unique<rtt::ai::stp::TestPlay>());

  plays.emplace_back(std::make_unique<rtt::ai::stp::play::AttackingPass>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::Attack>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::Halt>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendShot>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendPass>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefensiveStopFormation>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::AggressiveStopFormation>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementUs>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementThem>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::TimeOut>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThemPrepare>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUsPrepare>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThem>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUs>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUsPrepare>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThemPrepare>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::FreeKickThem>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUs>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThem>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallPossession>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallRisky>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::ReflectKick>());
  plays.emplace_back(std::make_unique<rtt::ai::stp::play::GenericPass>());
  playChecker.setPlays(plays);

}
void rtt::AI::decidePlay(rtt::world::World *_world) {
  playChecker.update(_world);

  // Here for manual change with the interface
  if(playDecider.interfacePlayChanged) {
    auto validPlays = playChecker.getValidPlays();
    currentPlay = playDecider.decideBestPlay(_world, validPlays);
    currentPlay->updateWorld(_world);
    currentPlay->initialize();
    playDecider.interfacePlayChanged = false;
  }

  // A new play will be chosen if the current play is not valid to keep
  if (!currentPlay || !currentPlay->isValidPlayToKeep(_world)) {
    auto validPlays = playChecker.getValidPlays();
    if (validPlays.empty()) {
      RTT_ERROR("No valid plays")
      currentPlay = playChecker.getPlayForName("Defend Shot"); //TODO Try out different default plays so both teams dont get stuck in Defend Shot when playing against yourself
      if (!currentPlay) {
        return;
      }
    } else {
      currentPlay = playDecider.decideBestPlay(_world, validPlays);
    }
    currentPlay->updateWorld(_world);
    currentPlay->initialize();
  }

  currentPlay->update();
  //mainWindow->updatePlay(currentPlay); TODO: send to interface
}
