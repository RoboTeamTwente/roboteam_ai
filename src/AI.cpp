//
// Created by rolf on 15-02-21.
//

#include "AI.h"
#include <roboteam_utils/normalize.h>
#include "utilities/GameStateManager.hpp"

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

rtt::AI::AI(int id) : settings(id) {
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

  world = std::make_unique<world::World>(settings.isYellow());
}
proto::AICommand rtt::AI::decidePlay() {
  //TODO: collect commands in ControlModule

  world::World * _world = world.get();//TODO: fix ownership, why are we handing out a raw pointer? Maybe const& to unique_ptr makes more sense
  playChecker.update(_world);

  // Here for manual change with the interface
//  if(playDecider.interfacePlayChanged) { //TODO: is this still relevant? We need to write interface handler anyways
//    auto validPlays = playChecker.getValidPlays();
//    currentPlay = playDecider.decideBestPlay(_world, validPlays);
//    currentPlay->updateWorld(_world);
//    currentPlay->initialize();
//    playDecider.interfacePlayChanged = false;
//  }

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
void rtt::AI::updateState(const proto::State& state) {


  updateSettingsReferee(state);

  //state has to be rotated to our perspective.

  //TODO: safety checks

  //Note these calls assume the proto field exist. Otherwise, all fields and subfields are initialized as empty!!
  auto worldMessage = state.last_seen_world();
  auto fieldMessage = state.field().field();
  if (!settings.isLeft()) {
    roboteam_utils::rotate(&worldMessage);
  }
  world->updateWorld(worldMessage);
  world->updateField(fieldMessage);
  world->updatePositionControl();
  //world->updateFeedback(feedbackMap); TODO fix feedback
}
void rtt::AI::updateSettingsReferee(const proto::State& state) {

  if(state.has_referee()){
    auto referee = state.referee();

    roboteam_utils::rotate(&referee); //this only rotates the ball placement marker.

    //TODO: put the following block in an if statement that checks whether we want to automatically listen to the referee or not
    //TODO: put the AI name into the settings
    {
      // Our name as specified by ssl-game-controller : https://github.com/RoboCup-SSL/ssl-game-controller/blob/master/config/engine.yaml
      //make sure this matches exactly!
      std::string ROBOTEAM_TWENTE = "RoboTeam Twente";
      bool color_changed = false;
      if (referee.yellow().name() == ROBOTEAM_TWENTE && !settings.isYellow()) {
        settings.setYellow(true);
        color_changed = true;
      } else if (referee.blue().name() == ROBOTEAM_TWENTE && settings.isYellow()) {
        settings.setYellow(false);
        color_changed = true;
      }
      bool we_are_left = !(referee.blue_team_on_positive_half() ^ settings.isYellow());
      //if we changed color or if we are playing on the wrong side, update sides
      if (color_changed || we_are_left != settings.isLeft()) {
        settings.setLeft(we_are_left);
        onSideOrColorChanged();
      }
    }
    ai::GameStateManager::setRefereeData(referee, world.get(),settings); //TODO: fix world *
  }
}
void rtt::AI::onSideOrColorChanged() {
  //TODO: reinitialize world, field, referee, and stop play
}
