#include "ApplicationManager.h"

#include <chrono>

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

    auto received_values = io->centralServerReceiveLastMessage();
    if(received_values.has_value()) {
      settings.updateValuesFromInterface(received_values.value());
      iface.handleUpdate(received_values.value());
    }

    //TODO: only send declarations when central server is reconnected

    std::vector<proto::Handshake> handshakes = {};
    if (const auto& decls = iface.getDeclarations().lock()) {
        proto::Handshake handshake;

        handshake.mutable_declarations()->CopyFrom(decls->toProto());

        handshakes.emplace_back(handshake);
    } else {
        RTT_ERROR("Can't get access to interface declarations!");
    }

    io->centralServerSend(handshakes);

    proto::AICommand command = ai->decidePlay();

    io->publishAICommand(command);

}
}  // namespace rtt
