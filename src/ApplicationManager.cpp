//
// Created by mrlukasbos on 14-1-19.
//


#include <roboteam_utils/Timer.h>
#include "utilities/Constants.h"
#include <bt/Node.hpp>
#include <ApplicationManager.h>
#include <utilities/GameStateManager.hpp>
#include <Settings/Settings.h>
#include <interface/api/Input.h>
#include <world/World.h>
#include <coach/GetBallCoach.h>
#include <coach/PassCoach.h>
#include <coach/defence/DefenceDealer.h>
#include <analysis/GameAnalyzer.h>
#include <coach/OffensiveCoach.h>
#include <include/roboteam_ai/world/Field.h>
#include "analysis/PlayChecker.h"
#include "include/roboteam_ai/analysis/PlaysObjects/Invariants/BallBelongsToUsInvariant.h"
#include "analysis/PlaysObjects/MyPlay.h"
namespace io = rtt::ai::io;
namespace ai = rtt::ai;
using Status = bt::Node::Status;

namespace rtt {

    /// Start running behaviour trees. While doing so, publish settings and log the FPS of the system
void ApplicationManager::start() {
    // create playcheck object here
    playcheck = rtt::ai::analysis::PlayChecker();

    // make sure we start in halt state for safety
    ai::GameStateManager::forceNewGameState(RefCommand::HALT);

    int amountOfCycles = 0;
    roboteam_utils::Timer t;
    t.loop([&]() {

        // This function runs the behaviour trees
        runOneLoopCycle();

        amountOfCycles++;

        // update the measured FPS, but limit this function call to only run 5 times/s at most
        int fpsUpdateRate = 5;
        t.limit([&]() {
            ai::interface::Input::setFps(amountOfCycles * fpsUpdateRate);
            amountOfCycles = 0;
        }, fpsUpdateRate);


        // publish settings, but limit this function call to only run 1 times/s at most
        t.limit([&]() {
            io::io.publishSettings(SETTINGS.toMessage());
        }, 1);

    }, ai::Constants::TICK_RATE());
}

/// Run everything with regard to behaviour trees
void ApplicationManager::runOneLoopCycle() {
    if (weHaveRobots && io::io.hasReceivedGeom) {
        ai::analysis::GameAnalyzer::getInstance().start();

        playcheck.update(rtt::ai::world::world, rtt::ai::world::field);

        updateTrees();
        updateCoaches();
        runKeeperTree();
        Status status = runStrategyTree();
        this->notifyTreeStatus(status);
    } else {
        std::this_thread::sleep_for(std::chrono::microseconds(100000));
    }
    weHaveRobots = ai::world::world->weHaveRobots();
    /*
    * This is a hack performed at the robocup.
    * It does a soft refresh when robots are not properly claimed by robotdealer.
    */
    checkForFreeRobots();
}

// Update the trees from the GameState
// The gamestate is usually altered by the interface or the referee
// or, in exceptional cases, by forcing it in the code (for example in the notifyTreeStatus() function below)
void ApplicationManager::updateTrees() {
    auto gameState = ai::GameStateManager::getCurrentGameState();
    std::string strategyName = gameState.strategyName;
    std::string keeperTreeName = gameState.keeperStrategyName;

    bool strategyChanged = oldStrategyName != strategyName;
    bool keeperStrategyChanged = oldKeeperTreeName != keeperTreeName;

    if (strategyChanged) {
        BTFactory::setCurrentTree(strategyName);
        oldStrategyName = strategyName;
    }

    if (keeperStrategyChanged) {
        BTFactory::setKeeperTree(keeperTreeName);
        oldKeeperTreeName = keeperTreeName;
    }

    if (keeperStrategyChanged || strategyChanged) {
        ai::robotDealer::RobotDealer::refresh();
    }
    ai::robotDealer::RobotDealer::setKeeperID(gameState.keeperId);
}

/// Tick the keeper tree if both the tree and keeper exist
void ApplicationManager::runKeeperTree() {
    keeperTree = BTFactory::getKeeperTree();
    if (keeperTree && ai::robotDealer::RobotDealer::keeperExistsInWorld()) {
        keeperTree->tick(ai::world::world, ai::world::field);
    }
}

/// Tick the strategy tree if the tree exists
Status ApplicationManager::runStrategyTree() {
    if (BTFactory::getCurrentTree() == "NaN") {
        std::cout << "NaN tree probably Halting" << std::endl;
          return Status::Waiting;
    }
    strategy = BTFactory::getTree(BTFactory::getCurrentTree());
    Status status = strategy->tick(ai::world::world, ai::world::field);
    return status;
}

/// Update the coaches information
void ApplicationManager::updateCoaches() const {
    auto coachesCalculationTime = roboteam_utils::Timer::measure([&](){
        ai::coach::getBallCoach->update();
        ai::coach::g_DefenceDealer.updateDefenderLocations();
        ai::coach::g_offensiveCoach.updateOffensivePositions();
        ai::coach::g_pass.updatePassProgression();
    });
    std::cout << "the coaches took: " << coachesCalculationTime.count() << " ms to calculate" << std::endl;
}

/// Terminate trees
void ApplicationManager::checkForShutdown() {
    // Terminate if needed
    if (strategy->getStatus() == Status::Running) {
        strategy->terminate(Status::Running);
    }
    ai::analysis::GameAnalyzer::getInstance().stop();
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

/// handle the status of a tree, and traverse to normal play when a tree either succeeds or fails.
void ApplicationManager::notifyTreeStatus(bt::Node::Status status) {
    switch (status) {
    case Status::Running:break;
    case Status::Success:
        std::cout << " === TREE SUCCESS -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        ai::GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
        break;
    case Status::Failure:
        std::cout << " === TREE FAILURE -> CHANGE TO NORMAL_PLAY_STRATEGY === " << std::endl;
        ai::GameStateManager::forceNewGameState(RefCommand::NORMAL_START);
      break;
    case Status::Waiting:
        std::cout << " === Status returned: Waiting === " << std::endl;
        break;
    }
}

} // rtt
