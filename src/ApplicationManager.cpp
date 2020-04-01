#include <ApplicationManager.h>
#include <coach/GetBallCoach.h>
#include <coach/OffensiveCoach.h>
#include <coach/PassCoach.h>
#include <coach/defence/DefenceDealer.h>
#include <interface/api/Input.h>
#include <roboteam_utils/Print.h>
#include <roboteam_utils/Timer.h>
#include <stp/new_plays/TestPlay.h>
#include <utilities/RobotDealer.h>

#include <utilities/GameStateManager.hpp>
#include <world_new/World.hpp>
#include <stp/new_plays/Halt.h>
#include "stp/new_plays/Pass.h"
#include "stp/new_plays/Attack.h"

#include "roboteam_utils/normalize.h"
#include "utilities/Constants.h"

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

    plays.emplace_back(std::make_unique<rtt::ai::stp::TestPlay>("Test"));
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Pass>("Pass"));
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Attack>("Attack"));
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::Halt>("Halt"));

    playChecker.setPlays(plays);

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

            /**
             * Comment/uncomment this line for new system (can't be used at the same time!)
             */
            decidePlay(world_new::World::instance());

            /**
             * Comment/uncomment these lines for old system (can't be used at the same time!)
             */
            // updateTrees();
            // updateCoaches(field);
            // runKeeperTree(field);
            // Status status = runStrategyTree(field);
            // this->notifyTreeStatus(status);
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
     * It does a soft refresh when robots are not properly claimed by robotdealer.
     */
    checkForFreeRobots();
}

/// Update the coaches information
void ApplicationManager::updateCoaches(const ai::world::Field &field) const {
    auto coachesCalculationTime = roboteam_utils::Timer::measure([&]() {
        ai::coach::getBallCoach->update(field);
        ai::coach::g_DefenceDealer.updateDefenderLocations(field);
        ai::coach::g_offensiveCoach.updateOffensivePositions(field);
        ai::coach::g_pass.updatePassProgression();
    });
    //    std::cout << "the coaches took: " << coachesCalculationTime.count() << " ms to calculate" << std::endl;
}

void ApplicationManager::checkForShutdown() {
    // Terminate if needed
    // TODO:
    //    if (strategy->getStatus() == Status::Running) {
//        strategy->terminate(Status::Running);
//    }
}

// Robotdealer hack to prevent robots from staying 'free' during play
void ApplicationManager::checkForFreeRobots() {
    if (ai::robotDealer::RobotDealer::hasFree()) {
        if (ticksFree++ > 10) {
            ai::robotDealer::RobotDealer::refresh();
        }
    } else {
        ticksFree = 0;
    }
}

void ApplicationManager::decidePlay(world_new::World *_world) {
    playChecker.update(_world);

    // A new play will be chosen if the current play is not valid to keep, or the roles are all finished, in which case the
    // play is considered finished
    if (!currentPlay || !currentPlay->isValidPlayToKeep(_world) || currentPlay->arePlayRolesFinished()) {
        currentPlay = playDecider.decideBestPlay(_world, playChecker.getValidPlays());
        currentPlay->updateWorld(_world);
        currentPlay->initialize();
    }

    currentPlay->update();
    mainWindow->updatePlay(currentPlay);
}

    ApplicationManager::ApplicationManager(ai::interface::MainWindow *mainWindow) {
        this->mainWindow = mainWindow;
    }
}  // namespace rtt
