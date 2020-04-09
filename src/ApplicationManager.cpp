#include <ApplicationManager.h>
#include <include/roboteam_ai/stp/new_plays_analysis/PassProblem.h>
#include <include/roboteam_ai/utilities/IOManager.h>
#include <interface/api/Input.h>
#include <roboteam_utils/Timer.h>
#include <roboteam_utils/normalize.h>

#include <utilities/GameStateManager.hpp>

#include "stp/new_plays_analysis/TestProblem.h"
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

    startArchipelago();

    int amountOfCycles = 0;
    roboteam_utils::Timer t;
    t.loop(
        [&]() {
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
            auto field = world_new::World::instance()->getField().value();

            /// If idle, setup islands if necessary and evolve again
            if (archipelago.status() == pagmo::evolve_status::idle) {
                RTT_WARNING("Archipelago idle, updating archipelago and evolving")
                updateArchipelago();
                archipelago.evolve(100);
            }

            /**
             * Comment/uncomment this line for new system (can't be used at the same time!)
             */
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
    /*
     * This is a hack performed at the robocup.
     */
    checkForFreeRobots();
}

void ApplicationManager::checkForShutdown() {
    // Terminate if needed
    // TODO:
    //    if (strategy->getStatus() == Status::Running) {
    //        strategy->terminate(Status::Running);
    //    }
}

void ApplicationManager::checkForFreeRobots() {
    // todo: replace this
    // basically just update tick count for how long robots have been free? i guess?
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

void ApplicationManager::startArchipelago() {
    /// clearing the archipelago
    archipelago = pagmo::archipelago{};

    /// Adding islands to the archipelago
    archipelago.push_back(pagmo::island{pagmo::algorithm{}, pagmo::population{}});
}

void ApplicationManager::updateArchipelago() {
    /// generating pass problem
    ai::stp::PassProblem passProblem{};
    passProblem.updateInfoForProblem(world_new::World::instance());
    auto passPopulation = pagmo::population(passProblem, 10,0);

    /// setting refreshed population
    archipelago[0].set_population(passPopulation);
}
}  // namespace rtt
