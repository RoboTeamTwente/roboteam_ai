#include <roboteam_utils/Print.h>

#include "STPManager.h"
#include "utilities/IOManager.h"
#include "world/World.hpp"

int main(int argc, char* argv[]) {
    if (argc != 2) {
        RTT_ERROR("Incorrect amount of arguments")
        RTT_INFO("Pass '0' as argument to indicate this is the primary AI, or anything else for a secondary AI")
        return 0;
    }

    RTT_INFO("\n",
             "                                           \n"
             "  ██████╗ ████████╗████████╗     █████╗ ██╗\n"
             "  ██╔══██╗╚══██╔══╝╚══██╔══╝    ██╔══██╗██║\n"
             "  ██████╔╝   ██║      ██║       ███████║██║\n"
             "  ██╔══██╗   ██║      ██║       ██╔══██║██║\n"
             "  ██║  ██║   ██║      ██║       ██║  ██║██║\n"
             "  ╚═╝  ╚═╝   ╚═╝      ╚═╝       ╚═╝  ╚═╝╚═╝\n"
             "                                           ")

    RTT_DEBUG("Debug prints enabled")

    rtt::ai::Constants::init();

    // get the id of the ai from the init
    int id = *argv[1] - '0';

    rtt::SETTINGS.init(id);

    // If primary AI, we start at being yellow on the left
    if (!rtt::SETTINGS.setYellow(rtt::SETTINGS.isPrimaryAI())) {
        RTT_ERROR("Could not obtain command publishing channel. Exiting...")
        return 0;
    }

    rtt::SETTINGS.setLeft(rtt::SETTINGS.isPrimaryAI());

    rtt::SETTINGS.setRobotHubMode(rtt::Settings::RobotHubMode::SIMULATOR);
    rtt::SETTINGS.setVisionIp("127.0.0.1");
    rtt::SETTINGS.setVisionPort(10006);
    rtt::SETTINGS.setRefereeIp("224.5.23.1");
    rtt::SETTINGS.setRefereePort(10003);
    rtt::SETTINGS.setRobothubSendIp("127.0.0.1");
    rtt::SETTINGS.setRobothubSendPort(20011);


    RTT_INFO("AI initialized as: ", (rtt::SETTINGS.isPrimaryAI() ? "PRIMARY" : "SECONDARY"))
    RTT_INFO("Starting as color: ", (rtt::SETTINGS.isYellow() ? "YELLOW" : "BLUE"))
    RTT_INFO("Playing on side: ", (rtt::SETTINGS.isLeft() ? "LEFT" : "RIGHT"))
    RTT_INFO("This AI will ", rtt::SETTINGS.isPrimaryAI() ? "" : "NOT ", "broadcast settings")

    if (!rtt::ai::io::io.init(rtt::SETTINGS.isPrimaryAI())) {
        RTT_ERROR("Failed to initialize IO Manager. Exiting...")
        return 0;
    }

    rtt::STPManager app;
    app.start();
}
