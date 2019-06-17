#include "Toggles.h"

namespace rtt {
namespace ai {
namespace interface {

std::vector<Toggle> Toggles::toggles = {
        {Visual::DEBUG,             showType::ALL_ROBOTS,       "Show debug values"},
        {Visual::BALL_DATA,         showType::ALL_ROBOTS,       "Show ball data"},
        {Visual::BALL_HANDLING,     showType::ALL_ROBOTS,       "Show ball handling"},
        {Visual::PATHFINDING,       showType::ALL_ROBOTS,       "Show final paths"},
        {Visual::PATHFINDING_DEBUG, showType::NO_ROBOTS,        "Show pathfinding tried paths"},
        {Visual::KEEPER,            showType::NO_ROBOTS,        "Show keeper points"},
        {Visual::INTERCEPT,         showType::ALL_ROBOTS,       "Show Interceptions"},
        {Visual::DEFENSE,           showType::ALL_ROBOTS,       "Show defensive location"},
        {Visual::SHOTLINES,         showType::ALL_ROBOTS,       "Show desired shotlines"},
        {Visual::OFFENSE,           showType::SELECTED_ROBOTS,  "Show offensive default locations"},

};

} // interface
} // ai
} // rtt
