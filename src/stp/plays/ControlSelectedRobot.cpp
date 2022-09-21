//
// Created by Emiel on 23-01-22.
// This play allows for a robot to be selected via the interface and sent to a specific location on the field
//

#include "stp/plays/ControlSelectedRobot.h"

#include "interface/api/Output.h"
#include "interface/widgets/mainWindow.h"
#include "roboteam_ai.h"
#include "stp/roles/passive/Formation.h"

namespace rtt::ai::stp::play {

ControlSelectedRobot::ControlSelectedRobot() : Play() {
    // The Formation role is suitable for this, since it does just GoToPos and Rotate
    roles = std::array<std::unique_ptr<Role>, rtt::ai::Constants::ROBOT_COUNT()>{std::make_unique<role::Formation>(role::Formation("robot"))};
}

uint8_t ControlSelectedRobot::score(const rtt::world::Field& field) noexcept {
    // Since this play is only used manually, there is no need for a score
    return 0;
}

Dealer::FlagMap ControlSelectedRobot::decideRoleFlags() const noexcept {
    Dealer::FlagMap flagMap;
    // Only set a flag if a robot is selected. This prevents a random robot from being assigned.
    if (current_robot_id != -1) flagMap.insert({"robot", {DealerFlagPriority::REQUIRED, {}, current_robot_id}});
    return flagMap;
}

bool ControlSelectedRobot::shouldEndPlay() noexcept {
    // A window is required to make this work. Ensure that the window exists
    assert(window != nullptr);
    // Get the selected robots from the field visualizer
    rtt::ai::interface::Visualizer *visualizer = window->getVisualizer();
    assert(visualizer != nullptr);
    const std::unordered_map<int, rtt::world::view::RobotView> &selected_robots = visualizer->getSelectedRobots();

    // If the currently controlled robot is still selected, then keep the play. If not, reset the play
    if (selected_robots.find(current_robot_id) == selected_robots.end()) {
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

    // If there are no robots selected, set current robot id to -1 and stop
    if (selected_robots.size() == 0) {
        current_robot_id = -1;
        return;
    }

    // If the currently controlled robot is not selected, then pick the first robot that is selected
    if (selected_robots.find(current_robot_id) == selected_robots.end()) {
        current_robot_id = selected_robots.begin()->first;
    }

    // Get the marker position on the field, and assign that position to the robot
    Vector2 position = rtt::ai::interface::Output::getInterfaceMarkerPosition();
    stpInfos["robot"].setPositionToMoveTo(position);
}

const char *ControlSelectedRobot::getName() { return "ControlSelectedRobot"; }

}  // namespace rtt::ai::stp::play