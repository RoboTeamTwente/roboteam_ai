//
// Created by mrlukasbos on 5-3-19.
//

#include "analysis/AnalysisReport.h"

namespace rtt {
namespace ai {
namespace analysis {

RobotDanger AnalysisReport::getRobotDangerForId(int id, bool ourTeam) {
    auto robotDangers = ourTeam ? ourRobotsSortedOnDanger : theirRobotSortedOnDanger;

    auto found = std::find_if(
        robotDangers.begin(), robotDangers.end(), [id](auto const& elem){ return elem.first->id == id; }
    );

    return found == robotDangers.end() ? RobotDanger{} : found->second;

    /**
     * std::find_if does this :)
     */
    // if (!robotDangers.empty()) {
    //     for (auto const &dangerPair : robotDangers) {
    //         if (dangerPair.first->id == id) {
    //             return dangerPair.second;
    //         }
    //     }
    // }

    // return {};
}
} // analysis
} // ai
} // rtt
