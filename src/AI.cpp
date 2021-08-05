//
// Created by rolf on 15-02-21.
//

#include "AI.h"
#include <chrono>

#include <roboteam_utils/Timer.h>
#include <roboteam_utils/normalize.h>
#include <stp/plays/referee_specific/TimeOut.h>

#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"
#include "control/ControlModule.h"

/**
 * Plays are included here
 */
#include "stp/plays/referee_specific/AggressiveStopFormation.h"
#include "stp/plays/offensive/Attack.h"
#include "stp/plays/offensive/AttackingPass.h"
#include "stp/plays/referee_specific/BallPlacementThem.h"
#include "stp/plays/referee_specific/BallPlacementUs.h"
#include "stp/plays/defensive/DefendPass.h"
#include "stp/plays/defensive/DefendShot.h"
#include "stp/plays/referee_specific/DefensiveStopFormation.h"
#include "stp/plays/referee_specific/FreeKickThem.h"
#include "stp/plays/offensive/GenericPass.h"
#include "stp/plays/contested/GetBallPossession.h"
#include "stp/plays/contested/GetBallRisky.h"
#include "stp/plays/referee_specific/Halt.h"
#include "stp/plays/referee_specific/KickOffThem.h"
#include "stp/plays/referee_specific/KickOffThemPrepare.h"
#include "stp/plays/referee_specific/KickOffUs.h"
#include "stp/plays/referee_specific/KickOffUsPrepare.h"
#include "stp/plays/referee_specific/PenaltyThem.h"
#include "stp/plays/referee_specific/PenaltyThemPrepare.h"
#include "stp/plays/referee_specific/PenaltyUs.h"
#include "stp/plays/referee_specific/PenaltyUsPrepare.h"
#include "stp/plays/ReflectKick.h"

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
  if(!world->getField().has_value() || settings.isPaused()){ //TODO: check if we *just* paused, and if so, send a stop signal to all ID's
    return proto::AICommand();
  }
  world::World * _world = world.get();//TODO: fix ownership, why are we handing out a raw pointer? Maybe const& to unique_ptr makes more sense

  playEvaluator.clearGlobalScores(); //reset all evaluations
  ai::stp::PositionComputations::calculatedScores.clear();
  ai::stp::PositionComputations::calculatedWallPositions.clear();

  playEvaluator.update(_world);
  playChecker.update(playEvaluator);



// A new play will be chosen if the current play is not valid to keep
    if (!currentPlay || !currentPlay->isValidPlayToKeep(playEvaluator)) {
        auto validPlays = playChecker.getValidPlays();
        ai::stp::gen::PlayInfos previousPlayInfo{};
        if(currentPlay) currentPlay->storePlayInfo(previousPlayInfo);

        if (validPlays.empty()) {
            RTT_ERROR("No valid plays")
            currentPlay = playChecker.getPlayForName("Defend Shot"); //TODO Try out different default plays so both teams dont get stuck in Defend Shot when playing against yourself
            if (!currentPlay) {
                return proto::AICommand();
            }
        } else {
            currentPlay = playDecider.decideBestPlay(validPlays, playEvaluator);
        }
        currentPlay->updateWorld(_world);
        currentPlay->initialize(previousPlayInfo);
    }
    currentPlay->update();

  auto commands = ai::control::ControlModule::sendAllCommands(settings); //TODO: no more statics
  proto::AICommand ai_command;
  for(const auto& command : commands){
    proto::RobotCommand * protoCommand = ai_command.mutable_commands()->Add();
    protoCommand->CopyFrom(command);
  }
  return ai_command;
  //mainWindow->updatePlay(currentPlay); TODO: send to interface
}
void rtt::AI::updateState(const proto::State& state) {


  updateSettingsReferee(state);

  //state has to be rotated to our perspective.

  //TODO: safety checks

  //Note these calls assume the proto field exist. Otherwise, all fields and subfields are initialized as empty!!
  if(state.has_last_seen_world()){
    auto worldMessage = state.last_seen_world();
    if (!settings.isLeft()) {
      roboteam_utils::rotate(&worldMessage);
    }
    world->updateWorld(worldMessage);
  }

  if(state.has_field()){
    auto fieldMessage = state.field().field();
    if (!settings.isLeft()) {
      roboteam_utils::rotate(&fieldMessage);
    }
    world->updateField(fieldMessage);
  }

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
proto::Handshake rtt::AI::getButtonDeclarations() const {
  return settings.getButtonDeclarations();
}
proto::Handshake rtt::AI::getSettingValues() const {
  return settings.getValues();
}
void rtt::AI::updateSettings(const proto::UiValues& values){
  return settings.updateValuesFromInterface(values);
}
