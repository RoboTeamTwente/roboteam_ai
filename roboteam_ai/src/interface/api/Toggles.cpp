#include "Toggles.h"

namespace rtt {
namespace ai {
namespace interface {

std::vector<Toggle> Toggles::toggles = {
        {Visual::DEBUG,       showType::ALL_ROBOTS,      "Show debug values"},
        {Visual::PATHFINDING, showType::SELECTED_ROBOTS, "Show pathfinding"},
        {Visual::KEEPER,      showType::NO_ROBOTS,       "Show keeper points"},
        {Visual::INTERCEPT,   showType::ALL_ROBOTS,      "Show Interceptions"},
        {Visual::DEFENSE,     showType::ALL_ROBOTS,      "Show defensive location"},
        {Visual::SHOTLINES,   showType::ALL_ROBOTS,      "Show desired shotlines"},
};

} // interface
} // ai
} // rtt
