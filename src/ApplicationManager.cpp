#include <ApplicationManager.h>
#include <interface/api/Input.h>
#include <roboteam_utils/Timer.h>
#include <roboteam_utils/normalize.h>
#include <stp/new_plays_analysis/PassProblem.h>
#include <utilities/IOManager.h>

#include <utilities/GameStateManager.hpp>

/**
 * Plays are included here
 */
#include "stp/new_plays/AggressiveFormation.h"
#include "stp/new_plays/Attack.h"
#include "stp/new_plays/BallPlacement.h"
#include "stp/new_plays/Defend.h"
#include "stp/new_plays/DefensiveFormation.h"
#include "stp/new_plays/Halt.h"
#include "stp/new_plays/Pass.h"
#include "stp/new_plays/TestPlay.h"
#include <pagmo/algorithms/pso.hpp>
#include <pagmo/algorithms/pso_gen.hpp>

namespace io = rtt::ai::io;
namespace ai = rtt::ai;

namespace rtt {

/// Start running behaviour trees. While doing so, publish settings and log the FPS of the system
void ApplicationManager::start() {
    // make sure we start in halt state for safety
    ai::GameStateManager::forceNewGameState(RefCommand::HALT, std::nullopt);
    RTT_INFO("Start looping")
    RTT_INFO("Waiting for field_data and robots...")

    setPlays();

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
            std::cout << "tick duration: "<< diff.count() << std::endl;
            std::cout << "tick allowed: " << 0.016 << std::endl;
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
                archipelago.evolve(100);
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

void ApplicationManager::setPlays() {
    plays.clear();

    plays.emplace_back(std::make_unique<rtt::ai::stp::TestPlay>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Pass>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Attack>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Halt>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Defend>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefensiveFormation>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::AggressiveFormation>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacement>());

    playChecker.setPlays(plays);
}

void ApplicationManager::updateArchipelago() {
    /// generating pass problem
    auto pso = pagmo::pso(20, 0.6, 0.6, 0.6, 0.6, 4, 2, 2, 1, 0);

    ai::stp::PassProblem passProblem{};
    passProblem.updateInfoForProblem(world_new::World::instance());
    auto passPopulation = pagmo::population(passProblem, 100, 0);

    /// clearing the archipelago
    archipelago = pagmo::archipelago{};

    /// Adding islands to the archipelago
    auto isl = pagmo::island{pagmo::algorithm{pagmo::pso_gen()}, passPopulation};
    archipelago.push_back(isl);
}
}  // namespace rtt
