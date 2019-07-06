#include "Toggles.h"

namespace rtt {
namespace ai {
namespace interface {

std::vector<Toggle> Toggles::toggles = {
        Toggle(Visual::DEBUG,             GeneralShowType::ON,             "Show debug values"),
        Toggle(Visual::BALL_DATA,         GeneralShowType::ON,             "Show ball data"),
        Toggle(Visual::BALL_HANDLING,     RobotShowType::ALL_ROBOTS,       "Show ball handling"),
        Toggle(Visual::PATHFINDING,       RobotShowType::ALL_ROBOTS,       "Show final paths"),
        Toggle(Visual::PATHFINDING_DEBUG, RobotShowType::NO_ROBOTS,        "Show pathfinding tried paths"),
        Toggle(Visual::KEEPER,            RobotShowType::NO_ROBOTS,        "Show keeper points"),
        Toggle(Visual::INTERCEPT,         RobotShowType::ALL_ROBOTS,       "Show Interceptions"),
        Toggle(Visual::DEFENSE,           RobotShowType::ALL_ROBOTS,       "Show defensive location"),
        Toggle(Visual::SHOTLINES,         RobotShowType::ALL_ROBOTS,       "Show desired shotlines"),
        Toggle(Visual::OFFENSE,           RobotShowType::SELECTED_ROBOTS,  "Show offensive default locations"),
        Toggle(Visual::BALLPLACEMENT,     RobotShowType::ALL_ROBOTS,       "Show ballplacement hints"),
        Toggle(Visual::AVOIDANCE,         RobotShowType::ALL_ROBOTS,       "Show offensive default locations")

};

} // interface
} // ai
} // rtt
