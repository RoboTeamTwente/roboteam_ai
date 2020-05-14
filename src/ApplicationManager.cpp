#include <ApplicationManager.h>
#include <interface/api/Input.h>
#include <roboteam_utils/Timer.h>
#include <roboteam_utils/normalize.h>
#include <stp/new_plays_analysis/PassProblem.h>
#include <utilities/IOManager.h>

#include <utilities/GameStateManager.hpp>

#include <pagmo/algorithms/pso.hpp>
#include <pagmo/islands/thread_island.hpp>
/**
 * Plays are included here
 */
#include "stp/new_plays/AggressiveFormation.h"
#include "stp/new_plays/Attack.h"
#include "stp/new_plays/Defend.h"
#include "stp/new_plays/Halt.h"
#include "stp/new_plays/BallPlacementUs.h"
#include "stp/new_plays/BallPlacementThem.h"
#include "stp/new_plays/DefensiveFormation.h"
#include "stp/new_plays/Pass.h"
#include "stp/new_plays/TestPlay.h"
#include "stp/new_plays/TimeOut.h"
#include "stp/new_plays/PenaltyThemPrepare.h"
#include "stp/new_plays/PenaltyUsPrepare.h"
#include "stp/new_plays/PenaltyThem.h"
#include "stp/new_plays/PenaltyUs.h"
#include "stp/new_plays/KickOffUsPrepare.h"
#include "stp/new_plays/KickOffThemPrepare.h"
#include "stp/new_plays/FreeKickThem.h"
#include "stp/new_plays/KickOffUs.h"
#include "stp/new_plays/KickOffThem.h"
#include "stp/new_plays/GetBallPossession.h"

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
    plays.emplace_back(std::make_unique<rtt::ai::stp::TestPlay>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Pass>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Attack>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Halt>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Defend>());
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
    playChecker.setPlays(plays);

    int amountOfCycles = 0;
    roboteam_utils::Timer t;
    t.loop(
        [&]() {
            auto begin = std::chrono::steady_clock::now();

            runOneLoopCycle();
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

            auto end = std::chrono::steady_clock::now();
            auto diff = std::chrono::duration<double>(end-begin);
            RTT_DEBUG("tick duration: ", diff.count())
            RTT_DEBUG("tick allowed: ", 0.016)
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

        if (!SETTINGS.isLeft()) {
            roboteam_utils::rotate(&worldMessage);
        }
        world_new::World::instance()->updateWorld(worldMessage);

        if (!world_new::World::instance()->getWorld()->getUs().empty()) {
            if (!robotsInitialized) {
                RTT_SUCCESS("Received robots, starting behaviour trees!")
            }
            robotsInitialized = true;

            world_new::World::instance()->updateField(fieldMessage);
            world_new::World::instance()->updatePositionControl();

            /// If idle, update the archipelago and evolve again
            if (archipelago.status() == pagmo::evolve_status::idle) {
                RTT_WARNING("Archipelago idle, updating and evolving")
                updateArchipelago();
                archipelago[0].evolve(200);
            }

            /// This call changes plays when necessary and ticks the currentPlay
            decidePlay(world_new::World::instance());
        } else {
            if (robotsInitialized) {
                RTT_WARNING("No robots found in world. Behaviour trees are not running")
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

    // A new play will be chosen if the current play is not valid to keep, or the roles are all finished, in which case the
    // play is considered finished
    if (!currentPlay || !currentPlay->isValidPlayToKeep(_world) || currentPlay->arePlayRolesFinished()) {
        auto validPlays = playChecker.getValidPlays();
        if (validPlays.empty()) {
            RTT_ERROR("No valid plays")
            // TODO: maybe we want to assign some default play (halt?) when there are no valid plays. Don't forget to cal initialize when we do!
            currentPlay = nullptr;
            return;
        }
        currentPlay = playDecider.decideBestPlay(_world, validPlays);
        currentPlay->updateWorld(_world);
        currentPlay->initialize(archipelago);
    }

    currentPlay->update();
    mainWindow->updatePlay(currentPlay);
}

ApplicationManager::ApplicationManager(ai::interface::MainWindow *mainWindow) { this->mainWindow = mainWindow; }



void ApplicationManager::updateArchipelago() {
    /// generating pass problem.
    int generations = 10;
    int popSize = 10;
    auto pso = pagmo::pso(generations);

    ai::stp::PassProblem passProblem{};

    auto field = world_new::World::instance()->getField().value();
    world_new::WorldData data = **world_new::World::instance()->getWorld();
    passProblem.updateInfoForProblem(data, field);

    auto passPopulation = pagmo::population(passProblem, popSize, 0);

    /// clearing the archipelago
    archipelago = pagmo::archipelago{};

    /// Adding islands to the archipelago
    auto isl = pagmo::island{pagmo::algorithm{pso}, passPopulation};
    archipelago.push_back(isl);
}
}  // namespace rtt
