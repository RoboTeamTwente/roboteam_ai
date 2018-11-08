//
// Created by rolf on 25-10-18.
//

#include "StrategyMapper.hpp"

namespace rtt {
namespace ai {
bool StrategyMapper::initialized = false;

std::shared_ptr<bt::BehaviorTree> StrategyMapper::mainStrategy;


/*
 * This is a mapping of Ref states to the appropriate strategies.
 * Any ref states that are not set, will fall back to the NORMAL_START strategy.
 *
 * Your JSON file should always be named as projectName/TreeName within this mapping.
 * The filename itself should be equal to the projectName, and should exist in the folder treeinterp/jsons.
 */
using namespace std::string_literals;
const std::map<RefGameState, boost::optional<std::string>> StrategyMapper::MAPPING = {
        ///////////////////////////////////////////////////////////////////////
        // States that direct to normal Play                                 //
        ///////////////////////////////////////////////////////////////////////

        {RefGameState::NORMAL_START, "bigjson/NormalPlay"s},
        {RefGameState::FORCED_START, "bigjson/009472f6-0d76-4db6-8161-a536bf497f89"s},

        ////////////////////////////////////////////////////
        // Ref states that have a specific implementation //
        ////////////////////////////////////////////////////

        {RefGameState::HALT, "rtt_dennis/HaltStrategy"s},
        {RefGameState::STOP, "rtt_anouk/StopStrat"s},
        {RefGameState::TIMEOUT_US, "rtt_anouk/StopStrat"s},
        {RefGameState::TIMEOUT_THEM, "rtt_anouk/StopStrat"s},
        {RefGameState::GOAL_US, "rtt_anouk/StopStrat"s},
        {RefGameState::GOAL_THEM, "rtt_anouk/StopStrat"s},

        {RefGameState::BALL_PLACEMENT_US, "rtt_anouk/BallPlacement_Strat"s},
        {RefGameState::BALL_PLACEMENT_THEM, "rtt_anouk/BallPlacementThemStrat"s},

        {RefGameState::PREPARE_KICKOFF_US, "rtt_emiel/PrepareKickoffUsStrategy"s},
        {RefGameState::PREPARE_KICKOFF_THEM, "rtt_emiel/PrepareKickoffThemStrategy"s},
        {RefGameState::PREPARE_PENALTY_US, "rtt_emiel/PreparePenaltyUsStrategy"s},
        {RefGameState::PREPARE_PENALTY_THEM, "rtt_emiel/PreparePenaltyThemStrategy"s},


        ///////////////////////////////////////////////////////////////////////////////
        // The following 4 refstates are in the twoState Switch.                     //
        // After they terminate NORMAL_START strategy should automatically be called //
        ///////////////////////////////////////////////////////////////////////////////

        {RefGameState::DIRECT_FREE_US, "rtt_jim/NormalPlay"s},// rtt_ewoud/FreeKickTakeStrategy
        {RefGameState::DIRECT_FREE_THEM, "rtt_anouk/PrepareDirectThem"s},// FreeKickDefenceStrategy
        {RefGameState::INDIRECT_FREE_US, "rtt_emiel/IndirectUsStrategy"s},// rtt_ewoud/FreeKickTakeStrategy
        {RefGameState::INDIRECT_FREE_THEM, "rtt_anouk/PrepareDirectThem"s},// FreeKickDefenceStrategy

        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        // Our custom refstates. These are always called after PREPARE_ RefGameStates when NORMAL_START is called (see Referee isTwoState)            //
        // After these strategies terminate, they will revert to NormalStart Strategy. These commands are created by us and never sent by the referee //
        ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

        // rtt_bob/KickoffWithRunStrategy
        {RefGameState::DO_KICKOFF, "rtt_bob/KickoffWithChipStrategy"s},
        {RefGameState::DEFEND_KICKOFF, "rtt_jim/KickOffDefenseStrat"s},
        {RefGameState::DEFEND_PENALTY, "rtt_emiel/PreparePenaltyThemStrategy"s},
        {RefGameState::DO_PENALTY, "rtt_jim/TakePenalty"s},
};
//TODO: change this? this is unintuitive behavior
std::shared_ptr<bt::BehaviorTree> StrategyMapper::getMainStrategy() {
    if (! initialized) init();
    return mainStrategy;
}
void StrategyMapper::init() {
    if (initialized) return;
    ROS_INFO_NAMED("StrategyMapper", "Initializing StrategyMapper..");

    // Construct RefStateManager and global BB
    std::shared_ptr<RefStateManager> rsm = std::make_shared<RefStateManager>();
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>(bt::Blackboard());//TODO: Create Global blackboard? @baris


    // Set the default tree name. Any tree that is not set will default to this.
    std::string defName;
    auto defNameIt = MAPPING.find(RefGameState::NORMAL_START);
    if (defNameIt != MAPPING.end() && defNameIt->second) {
        defName = *defNameIt->second;
    }
    else {
        ROS_ERROR_STREAM_NAMED("StrategyMapper",
                "Could not find a normal play strategy! " <<
                                                          "Please verify that the static MAPPING variable contains a normal play");
        return;
    }

    bt::BehaviorTree::Ptr defTree;
    std::pair<std::string, std::string> splitName = splitString(defName);
    std::string projectName = splitName.first;
    std::string treeName = splitName.second;
    if (BTFactory::treeRepo.find(projectName) != BTFactory::treeRepo.end()) {
        if (BTFactory::treeRepo[projectName].find(treeName) != BTFactory::treeRepo[projectName].end()) {
            defTree = std::make_shared<bt::BehaviorTree>(
                    BTFactory::treeRepo[projectName][treeName]); //Does this actually construct a pointer to the object in the map?
        }
        else {
            ROS_WARN_STREAM_NAMED("StrategyMapper",
                    "Could find the default project but not the default tree in the treeRepo");
        }
    }
    else { ROS_WARN_STREAM_NAMED("StrategyMapper", "Could not find the default project in the treeRepo"); }

    for (auto refState : ALL_REFSTATES) {
        auto stratNameOpt = MAPPING.at(refState); //name of desired strategy
        // if it is unset use default
        if (! stratNameOpt) {
            ROS_INFO_STREAM_NAMED("StrategyMapper",
                    "Strategy " << refStateToString(refState) << "is not set, using default strategy");
            rsm->AddStrategy(refState, defTree);
        }
        else {
            //This feels ugly. Is there a better way?
            auto stratName = *stratNameOpt;
            std::pair<std::string, std::string> stratSplitName = splitString(stratName);
            std::string stratProjectName = stratSplitName.first;
            std::string stratTreeName = stratSplitName.second;

            //try to find the desired strategy. If we find it, add it to the RefStateManager
            //TODO: also default the trees we cannot find in the treeRepo to NORMAL_START?
            if (BTFactory::treeRepo.find(stratProjectName) != BTFactory::treeRepo.end()) {
                if (BTFactory::treeRepo[stratProjectName].find(stratTreeName)
                        != BTFactory::treeRepo[stratProjectName].end()) {
                    // print first (for debugging purposes)
                    std::string refStateStr = refStateToString(refState);
                    refStateStr.resize(20, ' ');
                    ROS_INFO_STREAM_NAMED("StrategyMapper", "AddStrategy : " << refStateStr << " ->" << stratName);

                    // We find the node and add it to the refStateManager
                    auto node = std::make_shared<bt::BehaviorTree>(
                            BTFactory::treeRepo[stratProjectName][stratTreeName]);
                    rsm->AddStrategy(refState, node);
                }
                else {
                    ROS_WARN_STREAM_NAMED("StrategyMapper",
                            "The project: " << stratProjectName << " does not contain the tree: " << stratTreeName
                                            << " in the treeRepo.");
                }
            }
            else {
                ROS_WARN_STREAM_NAMED("StrategyMapper", "Could not find the project: " << stratProjectName
                                                                                       << " in the treeRepo. Using default strategy");
            }
        }

    }
    mainStrategy = std::make_shared<bt::BehaviorTree>();
    mainStrategy->SetRoot(rsm);
    initialized = true;
    ROS_INFO_STREAM_NAMED("StrategyMapper", "StrategyMapper Initialized!");
}
//Splits a string formatted abcd/efgh into strings abcd and efgh. Takes the first /
std::pair<std::string, std::string> StrategyMapper::splitString(std::string fullString) {
    std::string c = "/";
    std::string::size_type j = fullString.find(c);
    std::string projectName = fullString.substr(0, j);
    std::string treeName = fullString.substr(j + 1, fullString.length());
    return std::pair<std::string, std::string>(projectName, treeName);
}
}//ai
}//rtt