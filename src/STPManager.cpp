#include "STPManager.h"

#include <roboteam_utils/Timer.h>
#include <utilities/normalize.h>

#include <chrono>

#include "control/ControlModule.h"
#include "stp/PlayDecider.hpp"
#include "stp/PlayEvaluator.h"
#include "stp/computations/ComputationManager.h"
#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"

/**
 * Plays are included here
 */
#include "stp/plays/referee_specific/FormationPreHalf.h"
#include "stp/plays/defensive/DefendPass.h"
#include "stp/plays/defensive/DefendShot.h"
#include "stp/plays/defensive/KeeperKickBall.h"
#include "stp/plays/offensive/Attack.h"
#include "stp/plays/offensive/AttackingPass.h"
#include "stp/plays/referee_specific/AggressiveStopFormation.h"
#include "stp/plays/referee_specific/BallPlacementThem.h"
#include "stp/plays/referee_specific/BallPlacementUs.h"
#include "stp/plays/referee_specific/DefensiveStopFormation.h"
#include "stp/plays/referee_specific/FreeKickThem.h"
#include "stp/plays/referee_specific/FreeKickUsAtGoal.h"
#include "stp/plays/referee_specific/FreeKickUsPass.h"
#include "stp/plays/referee_specific/Halt.h"
#include "stp/plays/referee_specific/KickOffThem.h"
#include "stp/plays/referee_specific/KickOffThemPrepare.h"
#include "stp/plays/referee_specific/KickOffUs.h"
#include "stp/plays/referee_specific/KickOffUsPrepare.h"
#include "stp/plays/referee_specific/PenaltyThem.h"
#include "stp/plays/referee_specific/PenaltyThemPrepare.h"
#include "stp/plays/referee_specific/PenaltyUs.h"
#include "stp/plays/referee_specific/PenaltyUsPrepare.h"

namespace io = rtt::ai::io;
namespace ai = rtt::ai;

namespace rtt {

/// Start running behaviour trees. While doing so, publish settings and log the FPS of the system
void STPManager::start() {
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
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KeeperKickBall>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::DefensiveStopFormation>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::AggressiveStopFormation>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementUs>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::BallPlacementThem>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::TimeOut>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThemPrepare>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUsPrepare>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyThem>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::PenaltyUs>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUsPrepare>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThemPrepare>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::FreeKickThem>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::FreeKickUsAtGoal>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::FreeKickUsPass>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffUs>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::KickOffThem>());
    plays.emplace_back(std::make_unique<rtt::ai::stp::play::FormationPreHalf>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::GetBallRisky>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::ReflectKick>());
    // plays.emplace_back(std::make_unique<rtt::ai::stp::play::GenericPass>());

    // Set the pointer to world for all plays
    {
        auto const &[_, world] = world::World::instance();
        for (auto &play : plays) {
            play->setWorld(world);
        }
    }

    int amountOfCycles = 0;
    roboteam_utils::Timer stpTimer;
    stpTimer.loop(
        [&]() {
            // uncomment the 4 lines of code below to time and display the duration of each loop of the AI
            // std::chrono::steady_clock::time_point tStart = std::chrono::steady_clock::now();
            runOneLoopCycle();
            // std::chrono::steady_clock::time_point tStop = std::chrono::steady_clock::now();

            // auto loopcycleDuration = std::chrono::duration_cast<std::chrono::milliseconds>((tStop - tStart)).count();
            // RTT_DEBUG("Loop cycle duration = ", loopcycleDuration);
            amountOfCycles++;

            // update the measured FPS, but limit this function call to only run 5 times/s at most
            int fpsUpdateRate = 5;
            stpTimer.limit(
                [&]() {
                    ai::interface::Input::setFps(amountOfCycles * fpsUpdateRate);
                    amountOfCycles = 0;
                },
                fpsUpdateRate);

            // If this is primary AI, broadcast settings every second
            if (SETTINGS.isPrimaryAI()) {
                stpTimer.limit([&]() { io::io.publishSettings(SETTINGS); }, ai::Constants::SETTINGS_BROADCAST_RATE());
            }
        },
        ai::Constants::STP_TICK_RATE());
}

/// Run everything with regard to behaviour trees
void STPManager::runOneLoopCycle() {
    auto state = io::io.getState();
    if (state.has_field()) {
        if (!fieldInitialized) RTT_SUCCESS("Received first field message!")
        fieldInitialized = true;

        // Note these calls Assume the proto field exist. Otherwise, all fields and subfields are initialized as empty!!
        auto worldMessage = state.last_seen_world();
        auto fieldMessage = state.field().field();


        std::vector<proto::SSL_WrapperPacket> vision_packets(state.processed_vision_packets().begin(), state.processed_vision_packets().end());
        if (!SETTINGS.isLeft()) {
            roboteam_utils::rotate(&worldMessage);
            for (auto &packet : vision_packets) {
                roboteam_utils::rotate(&packet);  //
            }
        }
        mainWindow->updateProcessedVisionPackets(vision_packets);

        auto const &[_, world] = world::World::instance();
        world->updateWorld(worldMessage);

        if (!world->getWorld()->getUs().empty()) {
            if (!robotsInitialized) {
                RTT_SUCCESS("Received robots, starting STP!")
            }
            robotsInitialized = true;

            world->updateField(fieldMessage);
            world->updatePositionControl();

            decidePlay(world);

        } else {
            if (robotsInitialized) {
                RTT_WARNING("No robots found in world. STP is not running")
                robotsInitialized = false;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    } else {
        if (fieldInitialized) {
            RTT_WARNING("No field data present!")
            fieldInitialized = false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    rtt::ai::control::ControlModule::sendAllCommands();
}

void STPManager::decidePlay(world::World *_world) {
    ai::stp::PlayEvaluator::clearGlobalScores();  // reset all evaluations
    ai::stp::ComputationManager::clearStoredComputations();

    if (!currentPlay || rtt::ai::stp::PlayDecider::interfacePlayChanged || !currentPlay->isValidPlayToKeep()) {
        ai::stp::gen::PlayInfos previousPlayInfo{};
        if (currentPlay) currentPlay->storePlayInfo(previousPlayInfo);
        currentPlay = ai::stp::PlayDecider::decideBestPlay(_world, plays);
        currentPlay->updateField(_world->getField().value());
        currentPlay->initialize(previousPlayInfo);
    } else {
        currentPlay->updateField(_world->getField().value());
    }
    currentPlay->update();
    mainWindow->updatePlay(currentPlay);
}

STPManager::STPManager(ai::interface::MainWindow *mainWindow) { this->mainWindow = mainWindow; }
}  // namespace rtt
