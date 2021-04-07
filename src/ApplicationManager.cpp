#include "ApplicationManager.h"

#include <roboteam_utils/Timer.h>

#include "utilities/GameStateManager.hpp"
#include "utilities/IOManager.h"


namespace io = rtt::ai::io;
namespace ai = rtt::ai;

namespace rtt {

/// Start running behaviour trees. While doing so, publish settings and log the FPS of the system
void ApplicationManager::start(int id) {
  io = std::make_unique<io::IOManager>();

  rtt::ai::Constants::init();


  settings.setVisionIp("127.0.0.1");
  settings.setVisionPort(10006);
  settings.setRefereeIp("224.5.23.1");
  settings.setRefereePort(10003);

  io->init(id);

  // make sure we start in halt state for safety
    ai::GameStateManager::forceNewGameState(RefCommand::HALT, std::nullopt);
    RTT_INFO("Start looping")
    RTT_INFO("Waiting for field_data and robots...")
    ai = std::make_unique<AI>(id);

    roboteam_utils::Timer t;
    t.loop(
        [&]() {
            auto start = std::clock();
            runOneLoopCycle();

            RTT_WARNING("Time: ", (std::clock() - start) / (double)(CLOCKS_PER_SEC / 1000), " ms")
            RTT_WARNING("Time allowed: 16 ms")

            // publish settings, but limit this function call to only run 1 times/s at most
            t.limit([&]() { io->publishSettings(settings.toMessage()); }, 1);
        },
        ai::Constants::TICK_RATE());
}

/// Run everything with regard to behaviour trees
void ApplicationManager::runOneLoopCycle() {
    auto state = io->getState();
    ai->updateState(state);

    proto::AICommand command = ai->decidePlay();

    io->publishAICommand(command);
    io->handleCentralServerConnection();
}
}  // namespace rtt
