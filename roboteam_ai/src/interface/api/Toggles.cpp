#include "Toggles.h"

namespace rtt {
namespace ai {
namespace interface {

std::vector<Toggle> Toggles::toggles = {
        {Visual::DEBUG,             RobotShowType::ALL_ROBOTS,       "Show debug values"},
        {Visual::BALL_DATA,         GeneralShowType::ON,             "Show ball data"},
        {Visual::PATHFINDING,       RobotShowType::ALL_ROBOTS,       "Show final paths"},
        {Visual::PATHFINDING_DEBUG, RobotShowType::NO_ROBOTS,        "Show pathfinding tried paths"},
        {Visual::KEEPER,            RobotShowType::NO_ROBOTS,        "Show keeper points"},
        {Visual::INTERCEPT,         RobotShowType::ALL_ROBOTS,       "Show Interceptions"},
        {Visual::DEFENSE,           RobotShowType::ALL_ROBOTS,       "Show defensive location"},
        {Visual::SHOTLINES,         RobotShowType::ALL_ROBOTS,       "Show desired shotlines"},
        {Visual::OFFENSE,           RobotShowType::SELECTED_ROBOTS,  "Show offensive default locations"},

};

} // interface
} // ai
} // rtt
