#include <ApplicationManager.h>
#include <include/roboteam_ai/utilities/IOManager.h>
#include <roboteam_utils/Timer.h>
#include <roboteam_utils/normalize.h>

#include <utilities/GameStateManager.hpp>

/**
 * Plays are included here
 */
#include "stp/plays/AggressiveFormation.h"
#include "stp/plays/Attack.h"
#include "stp/plays/AttackingPass.h"
#include "stp/plays/BallPlacementThem.h"
#include "stp/plays/BallPlacementUs.h"
#include "stp/plays/DefendPass.h"
#include "stp/plays/DefendShot.h"
#include "stp/plays/DefensiveFormation.h"
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
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Attack>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Halt>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendShot>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefendPass>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefensiveFormation>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::AggressiveFormation>());
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

    int amountOfCycles = 0;
    roboteam_utils::Timer t;
    t.loop(
        [&]() {
            auto start = std::clock();
            runOneLoopCycle();

            RTT_WARNING("Time: ", (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000), " ms")
            RTT_WARNING("Time allowed: 16 ms")

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
    if (io::io.hasReceivedGeom) {
        if (!fieldInitialized) RTT_SUCCESS("Received first field message!")
        fieldInitialized = true;

        auto fieldMessage = io::io.getGeometryData().field();
        auto worldMessage = io::io.getWorldState();
        auto feedbackMap = io::io.getFeedbackDataMap();

        if (!SETTINGS.isLeft()) {
            roboteam_utils::rotate(&worldMessage);
        }
        auto const& [_, world] = world_new::World::instance();
        world->updateWorld(worldMessage);

        if (!world->getWorld()->getUs().empty()) {
            if (!robotsInitialized) {
                RTT_SUCCESS("Received robots, starting STP!")
            }
            robotsInitialized = true;

            world->updateField(fieldMessage);
            world->updatePositionControl();
            world->updateFeedback(feedbackMap);

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
}

void ApplicationManager::decidePlay(world_new::World *_world) {
    playChecker.update(_world);

    // A new play will be chosen if the current play is not valid to keep
    if (!currentPlay || !currentPlay->isValidPlayToKeep(_world)) {
        auto validPlays = playChecker.getValidPlays();
        if (validPlays.empty()) {
            RTT_ERROR("No valid plays")
            currentPlay = playChecker.getPlayForName("Defend Shot");
            if(!currentPlay) {
                return;
            }
        }
        else {
            currentPlay = playDecider.decideBestPlay(_world, validPlays);
        }
        currentPlay->updateWorld(_world);
        currentPlay->initialize();
    }

    currentPlay->update();
    mainWindow->updatePlay(currentPlay);
}

ApplicationManager::ApplicationManager(ai::interface::MainWindow *mainWindow) { this->mainWindow = mainWindow; }
}  // namespace rtt
