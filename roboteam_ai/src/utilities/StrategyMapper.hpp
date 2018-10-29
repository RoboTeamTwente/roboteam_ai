//
// Created by rolf on 25-10-18.
//

#ifndef ROBOTEAM_AI_STRATEGYMAPPER_HPP
#define ROBOTEAM_AI_STRATEGYMAPPER_HPP
#include "RefStateManager.hpp"
#include "Referee.hpp"
#include "../../src/bt/bt.hpp"
#include "../../src/treeinterp/BTFactory.h"

namespace rtt {
namespace ai {
/// \brief Maps the trees in BTFactory to the RefStateManager and checks if this all goes correctly.
class StrategyMapper {
private:
    static std::shared_ptr<bt::BehaviorTree> mainStrategy;
    static void init();
    static bool initialized;
    static std::pair<std::string,std::string> splitString(std::string fullString);
public:
    StrategyMapper()=delete;
    static std::shared_ptr<bt::BehaviorTree> getMainStrategy();
    // SET THIS IN THE CPP!!
    static const std::map<RefGameState, boost::optional<std::string>> MAPPING;

    };
}
}
#endif //ROBOTEAM_AI_STRATEGYMAPPER_HPP
