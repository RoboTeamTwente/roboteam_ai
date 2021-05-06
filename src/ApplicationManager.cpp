#include "ApplicationManager.h"

#include <roboteam_utils/Timer.h>
#include <roboteam_utils/normalize.h>

#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"
#include "control/ControlModule.h"

/**
 * Plays are included here
 */
//#include "include/roboteam_ai/stp/plays/referee_specific/AggressiveStopFormation.h"
//#include "include/roboteam_ai/stp/plays/offensive/Attack.h"
#include "include/roboteam_ai/stp/plays/offensive/AttackingPass.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/BallPlacementThem.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/BallPlacementUs.h"
//#include "include/roboteam_ai/stp/plays/defensive/DefendPass.h"
//#include "include/roboteam_ai/stp/plays/defensive/DefendShot.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/DefensiveStopFormation.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/FreeKickThem.h"
//#include "include/roboteam_ai/stp/plays/offensive/GenericPass.h"
#include "include/roboteam_ai/stp/plays/contested/GetBallPossession.h"
//#include "include/roboteam_ai/stp/plays/contested/GetBallRisky.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/Halt.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/KickOffThem.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/KickOffThemPrepare.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/KickOffUs.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/KickOffUsPrepare.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/PenaltyThem.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/PenaltyThemPrepare.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/PenaltyUs.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/PenaltyUsPrepare.h"
//#include "stp/plays/ReflectKick.h"
//#include "stp/plays/TestPlay.h"
//#include "include/roboteam_ai/stp/plays/referee_specific/TimeOut.h"

namespace io = rtt::ai::io;
namespace ai = rtt::ai;

namespace rtt {

/// Start running behaviour trees. While doing so, publish settings and log the FPS of the system
void ApplicationManager::start() {
    // make sure we start in halt state for safety
    ai::GameStateManager::forceNewGameState(RefCommand::HALT, std::nullopt);
    RTT_INFO("Start looping")
    RTT_INFO("Waiting for field_data and robots...")

    plays = std::vector<std::unique_ptr<rtt::ai::stp::Play>>{};

    /// This play is only used for testing purposes, when needed uncomment this play!
    // plays.emplace_back(std::make_unique<rtt::ai::stp::TestPlay>());

    plays.emplace_back(std::make_unique<rtt::ai::stp::play::AttackingPass>());
    //plays.emplace_back(std::make_unique<rtt::ai::stp::play::Attack>());
    //plays.emplace_back(std::make_unique<rtt::ai::stp::play::Halt>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendShot>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendPass>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefensiveStopFormation>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::AggressiveStopFormation>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementUs>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementThem>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::TimeOut>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThemPrepare>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUsPrepare>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThem>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUs>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUsPrepare>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThemPrepare>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::FreeKickThem>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUs>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThem>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallPossession>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallRisky>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::ReflectKick>());
//    plays.emplace_back(std::make_unique<rtt::ai::stp::play::GenericPass>());
    playChecker.setPlays(plays);

    int amountOfCycles = 0;
    roboteam_utils::Timer t;
    t.loop(
        [&]() {
            auto start = std::clock();
            runOneLoopCycle();

            //RTT_WARNING("Time: ", (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000), " ms")
            //RTT_WARNING("Time allowed: 16 ms")

            amountOfCycles++;

            // update the measured FPS, but limit this function call to only run 5 times/s at most
            int fpsUpdateRate = 5;
            t.limit(
                [&]() {
                    ai::interface::Input::setFps(amountOfCycles * fpsUpdateRate);
                    amountOfCycles = 0;
                },
                fpsUpdateRate);

            // publish settings, but limit this function call to only run 1 times/s at most
            t.limit([&]() { io::io.publishSettings(SETTINGS.toMessage()); }, 1);
        },
        ai::Constants::TICK_RATE());
}

/// Run everything with regard to behaviour trees
void ApplicationManager::runOneLoopCycle() {
    auto state = io::io.getState();
    if (state.has_field()) {
        if (!fieldInitialized) RTT_SUCCESS("Received first field message!")
        fieldInitialized = true;

        //Note these calls Assume the proto field exist. Otherwise, all fields and subfields are initialized as empty!!
        auto worldMessage = state.last_seen_world();
        auto fieldMessage = state.field().field();
        if (!SETTINGS.isLeft()) {
            roboteam_utils::rotate(&worldMessage);
        }
        auto const &[_, world] = world::World::instance();
        world->updateWorld(worldMessage);

        if (!world->getWorld()->getUs().empty()) {
            if (!robotsInitialized) {
                RTT_SUCCESS("Received robots, starting STP!")
            }
            robotsInitialized = true;


            world->updateField(fieldMessage);
            world->updatePositionControl();
            //world->updateFeedback(feedbackMap);

            decidePlay(world);

        } else {
            if (robotsInitialized) {
                RTT_WARNING("No robots found in world. STP is not running")
                robotsInitialized = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    } else {
        if (fieldInitialized) {
            RTT_WARNING("No field data present!")
            fieldInitialized = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    rtt::ai::control::ControlModule::sendAllCommands();
    io::io.handleCentralServerConnection();
}

void ApplicationManager::decidePlay(world::World *_world) {
    //TODO make a clear function
    playEvaluator.clearGlobalScores(); //reset all evaluations
    ai::stp::PositionComputations::calculatedScores.clear();
    ai::stp::PositionComputations::calculatedWallPositions.clear();

    playEvaluator.update(_world);
    playChecker.update(_world, playEvaluator);

    // Here for manual change with the interface
    if(rtt::ai::stp::PlayDecider::interfacePlayChanged) {
        auto validPlays = playChecker.getValidPlays();
        rtt::ai::stp::Play::PlayInfos previousPlayInfo{};
        if(currentPlay) currentPlay->storePlayInfo(previousPlayInfo);

        //Before a new play is possibly chosen: save all info of current Play that is necessary for a next Play
        currentPlay = playDecider.decideBestPlay(validPlays, playEvaluator);
        currentPlay->updateWorld(_world);
        currentPlay->initialize(previousPlayInfo);
        rtt::ai::stp::PlayDecider::interfacePlayChanged = false;
    }

    // A new play will be chosen if the current play is not valid to keep
    if (!currentPlay || !currentPlay->isValidPlayToKeep(playEvaluator)) {
        auto validPlays = playChecker.getValidPlays();
        rtt::ai::stp::Play::PlayInfos previousPlayInfo{};
        if(currentPlay) currentPlay->storePlayInfo(previousPlayInfo);

        if (validPlays.empty()) {
            RTT_ERROR("No valid plays")
            currentPlay = playChecker.getPlayForName("Defend Shot"); //TODO Try out different default plays so both teams dont get stuck in Defend Shot when playing against yourself
            if (!currentPlay) {
                return;
            }
        } else {
            currentPlay = playDecider.decideBestPlay(validPlays, playEvaluator);
        }
        currentPlay->updateWorld(_world);
        currentPlay->initialize(previousPlayInfo);
    }
    currentPlay->update();
    mainWindow->updatePlay(currentPlay);
}

ApplicationManager::ApplicationManager(ai::interface::MainWindow *mainWindow) { this->mainWindow = mainWindow; }
}  // namespace rtt
