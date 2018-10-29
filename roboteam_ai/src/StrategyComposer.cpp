//
// Created by rolf on 10-10-18.
//

#include "StrategyComposer.hpp"
//TODO: implement new factories/tree interpreter.
namespace b = boost;

namespace rtt {

bool StrategyComposer::initialized = false;

std::shared_ptr<bt::BehaviorTree> StrategyComposer::mainStrategy;

using namespace std::string_literals;

/*
 * This is a mapping of Ref states to the appropriate strategies.
 * Any UNSET ref states will fall back to the NORMAL_PLAY strategy.
 * If the NORMAL_PLAY strategy is UNSET, bad things will happen.
 *
 * ////////////////////////////////
 * // Don't forget the s suffix! //
 * ////////////////////////////////
 *
 */
const std::map<RefState, b::optional<std::string>> StrategyComposer::MAPPING = {
        ///////////////////////////////////////////////////////////////////////
        // Explicitly unused states that should redirect towards normal play //
        ///////////////////////////////////////////////////////////////////////

        {RefState::NORMAL_START, "rtt_jim/NormalPlay"s},
        {RefState::FORCED_START, "rtt_jim/NormalPlay"s},

        ////////////////////////////////////////////////////
        // Ref states that have a specific implementation //
        ////////////////////////////////////////////////////

        {RefState::HALT, "rtt_dennis/HaltStrategy"s},
        {RefState::STOP, "rtt_anouk/StopStrat"s},
        {RefState::PREPARE_KICKOFF_US, "rtt_emiel/PrepareKickoffUsStrategy"s},
        {RefState::PREPARE_KICKOFF_THEM, "rtt_emiel/PrepareKickoffThemStrategy"s},
        {RefState::PREPARE_PENALTY_US, "rtt_emiel/PreparePenaltyUsStrategy"s},
        {RefState::PREPARE_PENALTY_THEM, "rtt_emiel/PreparePenaltyThemStrategy"s},

        // rtt_ewoud/FreeKickTakeStrategy
        {RefState::DIRECT_FREE_US, "rtt_jim/NormalPlay"s},

        // FreeKickDefenceStrategy
        {RefState::DIRECT_FREE_THEM, "rtt_anouk/PrepareDirectThem"s},

        // rtt_ewoud/FreeKickTakeStrategy
        {RefState::INDIRECT_FREE_US, "rtt_emiel/IndirectUsStrategy"s},

        // FreeKickDefenceStrategy
        {RefState::INDIRECT_FREE_THEM, "rtt_anouk/PrepareDirectThem"s},
        {RefState::TIMEOUT_US, "rtt_anouk/StopStrat"s},
        {RefState::TIMEOUT_THEM, "rtt_anouk/StopStrat"s},
        {RefState::GOAL_US, "rtt_anouk/StopStrat"s},
        {RefState::GOAL_THEM, "rtt_anouk/StopStrat"s},
        {RefState::BALL_PLACEMENT_US, "rtt_anouk/BallPlacement_Strat"s},
        {RefState::BALL_PLACEMENT_THEM, "rtt_anouk/BallPlacementThemStrat"s},

        //////////////////////////
        // Our custom refstates //
        //////////////////////////

        // rtt_bob/KickoffWithRunStrategy
        {RefState::DO_KICKOFF, "rtt_bob/KickoffWithChipStrategy"s},
        {RefState::DEFEND_KICKOFF, "rtt_jim/KickOffDefenseStrat"s},
        {RefState::DEFEND_PENALTY, "rtt_emiel/PreparePenaltyThemStrategy"s},
        {RefState::DO_PENALTY, "rtt_jim/TakePenalty"s},

        // SHOULD BE REMOVED
        {RefState::NORMAL_PLAY, "rtt_jim/NormalPlay"s},
};

std::shared_ptr<bt::BehaviorTree> StrategyComposer::getMainStrategy() {
    if (! initialized) init();
    return mainStrategy;
}

void StrategyComposer::init() {
    // Return if not initialized
    if (initialized) return;

    ROS_INFO_NAMED("StrategyComposer", "Initializing StrategyComposer..");

    // Construct the global bb and the refstate switch
    bt::Blackboard::Ptr bb = std::make_shared<bt::Blackboard>(bt::Blackboard());
    std::shared_ptr<RefStateSwitch> rss = std::make_shared<RefStateSwitch>();

    //TODO: Fix  factory generation to modern factories.
    // Get the factory that stores all the behavior trees
    namespace f = ::rtt::factories;
    auto const &repo = f::getRepo < f::Factory < bt::BehaviorTree >> ();
    // Get the default tree factory map entry
    std::string defName;
    auto defNameIt = MAPPING.find(RefState::NORMAL_START);
    if (defNameIt != MAPPING.end() && defNameIt->second) {
        defName = *defNameIt->second;
    }
    else {
        ROS_ERROR_STREAM_NAMED("StrategyComposer",
                "Could not find a normal play strategy! " <<
                                                          "Please verify that the static MAPPING variable contains a normal play");
        return;
    }

    auto defIt = repo.find(defName);

    // If it exists, set it as default
    bt::BehaviorTree::Ptr def;
    if (defIt != repo.end()) {
        def = defIt->second("", bb);
    }
    else {
        ROS_ERROR_STREAM_NAMED("StrategyComposer",
                "Could not find a tree for default strategy tree \"" <<
                                                                     defName <<
                                                                     "\". Possibly \"refresh_b3_projects.sh\" needs to be run "
                                                                     <<
                                                                     "or a non-existent tree was selected.\n");
        return;
    }


    //TODO: fix traversing trees to modern version.
    for (auto refState : ALL_REFSTATES) {
        // Get the name of the desired strategy
        auto stratNameOpt = MAPPING.at(refState);

        // If it's unset use the default
        if (! stratNameOpt) {
            std::cout << "Forwarding tree: " << refStateToString(refState) << "\n";
            Forwarder fw(bb, def);
            // rss->AddChild(std::make_shared<Forwarder>(fw));
            rss->AddStrategy(refState, std::make_shared<Forwarder>(fw));
        }
        else {
            auto stratName = *stratNameOpt;
            // Else try to find the desired strategy tree
            auto stratIt = repo.find(stratName);

            if (stratIt != repo.end()) {
                // If so, set it
                auto node = stratIt->second("", bb);
                std::string refStateStr = refStateToString(refState);
                refStateStr.resize(20, ' ');
                ROS_INFO_STREAM_NAMED("StrategyComposer", "AddStrategy : " << refStateStr << " -> " << stratName);
                rss->AddStrategy(refState, node);
            }
            else {
                // Else it's not there, abort!
                std::cerr << "Could not find a tree for \""
                          << stratName
                          << "\" for refstate: \""
                          << refStateToString(refState)
                          << "\". Possibly \"refresh_b3_projects.sh\" needs to be run "
                          << "or a non-existent tree was selected.\n";
                return;
            }
        }

    }

    // The main strategy is now properly initialized
    mainStrategy = std::make_shared<bt::BehaviorTree>();
    mainStrategy->SetRoot(rss);
    initialized = true;
    ROS_INFO_NAMED("StrategyComposer", "StrategyComposer initialized");

}

StrategyComposer::Forwarder::Forwarder(bt::Blackboard::Ptr bb, bt::Node::Ptr target)
        :bt::Leaf(bb),
         target(target) { }

bt::Node::Status StrategyComposer::Forwarder::Update() { return target->Update(); }

void StrategyComposer::Forwarder::Initialize() { target->Initialize(); }

void StrategyComposer::Forwarder::Terminate(bt::Node::Status status) { target->Terminate(status); }

}
