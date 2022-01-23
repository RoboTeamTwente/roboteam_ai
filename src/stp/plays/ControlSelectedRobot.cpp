//
// Created by Emiel on 23-01-22.
//

#include "stp/plays/ControlSelectedRobot.h"

#include "interface/widgets/mainWindow.h"
#include "stp/roles/passive/Formation.h"
#include "roboteam_ai.h"
#include "interface/api/Output.h"

namespace rtt::ai::stp::play {

ControlSelectedRobot::ControlSelectedRobot() : Play() {
    std::cout << "[ControlSelectedRobot] New" << std::endl;

    roles = std::array< std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT() > {
        std::make_unique<role::Formation>( role::Formation("robot") )
    };
}

uint8_t ControlSelectedRobot::score(PlayEvaluator &playEvaluator) noexcept {
    std::cout << "[ControlSelectedRobot::score]" << std::endl;
    return 0;
}

Dealer::FlagMap ControlSelectedRobot::decideRoleFlags() const noexcept {
    std::cout << "[ControlSelectedRobot::decideRoleFlags]" << std::endl;
    Dealer::FlagMap flagMap;
    if(current_robot_id != -1)
        flagMap.insert({"robot", {DealerFlagPriority::REQUIRED, {}, current_robot_id} });
    return flagMap;
}

bool ControlSelectedRobot::isValidPlayToKeep(PlayEvaluator &playEvaluator) noexcept {
    // A window is required to make this work. Ensure that the window exists
    assert(window != nullptr);
    // Get the selected robots from the field visualizer
    rtt::ai::interface::Visualizer *visualizer = window->getVisualizer();
    assert(visualizer != nullptr);
    const std::unordered_map<int, rtt::world::view::RobotView> &selected_robots = visualizer->getSelectedRobots();

    // If the currently controlled robot is still selected, then keep the play. If not, reset the play
    if(selected_robots.find(current_robot_id) == selected_robots.end()){
        std::cout << "[isValidPlayToKeep] " << (selected_robots.find(current_robot_id) != selected_robots.end()) << std::endl;
        current_robot_id = -1;
        return false;
    }
    return true;
}

void ControlSelectedRobot::calculateInfoForRoles() noexcept {
    // A window is required to make this work. Ensure that the window exists
    assert(window != nullptr);
    // Get the selected robots from the field visualizer
    rtt::ai::interface::Visualizer *visualizer = window->getVisualizer();
    assert(visualizer != nullptr);
    const std::unordered_map<int, rtt::world::view::RobotView> &selected_robots = visualizer->getSelectedRobots();

    // If there are no robots selected, set id to -1 and stop
    if(selected_robots.size() == 0){
        current_robot_id = -1;
        return;
    }

    // If the currently controlled robot is not selected, then pick the first robot that is selected and redistribute roles
    if(selected_robots.find(current_robot_id) == selected_robots.end()){
        current_robot_id = selected_robots.begin()->first;
    }

    Vector2 position = rtt::ai::interface::Output::getInterfaceMarkerPosition();
    stpInfos["robot"].setPositionToMoveTo(position);
}

const char* ControlSelectedRobot::getName() {
    std::cout << "[ControlSelectedRobot::getName]" << std::endl;
    return "ControlSelectedRobot";
}


}