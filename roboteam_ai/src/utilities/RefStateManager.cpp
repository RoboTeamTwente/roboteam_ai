//
// Created by rolf on 23-10-18.
//

// The RefStateManager class. Update() gets the latest refereecommand and selects the right tree.
// Trees are added through the addStrategy function(), which is called in StrategyMapper
#include "RefStateManager.hpp"


namespace rtt {
namespace ai {
    RefStateManager::RefStateManager() : finishedOnce(false), needToInitialize(false), startedNewStrategy(false),
                                        lastKnownBotCount(-1) {}

    void RefStateManager::AddStrategy(RefGameState refGameState, Node::Ptr child) {
        refStateStrategies[refGameState] = child;
    }


    /**
     * (re)starts the right strategy tree if needed (change in number of robots, change in referee command)
     * @returns Status::Running
     */
    bt::Node::Status RefStateManager::Update() {

        // If there is no referee command yet, do nothing
        if (!Referee::hasReceivedFirstCommand()) {
            ROS_WARN_STREAM_NAMED("RefStateManager",
                                  "Have not yet received a ref command, so not executing any strategy tree.");
            return Status::Running;
        }

        const roboteam_msgs::World &world = World::get_world();

        // === If the number of robots change, restart the current strategy === //
        // Get the number of bots in the world
        unsigned botCount = world.us.size();
        // If the amount of bots changed
        if (botCount != lastKnownBotCount) {
            ROS_INFO_STREAM_NAMED("RefStateManager",
                                  "Botcount changed from " << lastKnownBotCount << " to " << botCount);
            // Update lastKnownBotCount
            lastKnownBotCount = world.us.size();
            // Get the current strategy tree
            if (auto child = getCurrentChild()) {
                // Terminate the strategy tree
                child->Terminate(Status::Invalid);
                // Restart the strategy tree
                child->Initialize();
            }
        }


        startedNewStrategy = false;
        auto cmd = Referee::getState();

        // === If the referee command changed, then terminate current tree and check for TwoState === //
        if (currentCmd != cmd) {

            // If there is a new command, terminate the current strategy
            if (currentCmd) {
                getCurrentChild()->Terminate(
                        getCurrentChild()->getStatus()); // getCurrentChild() automatically returns right tree based on previousCmd and currentCmd
            }

            // Update the previous and new command. This causes getCurrentChild() to return a different tree
            previousCmd = currentCmd;
            currentCmd = cmd;

            // Check if the switch is a TwoState switch
            if (Referee::isTwoState(previousCmd, *currentCmd)) {
                ROS_DEBUG_STREAM_NAMED("RefStateManager", "TwoState detected");
                finishedOnce = false;
            }

            needToInitialize = true;
            startedNewStrategy = true;

            if (previousCmd && currentCmd) {
                ROS_INFO_STREAM_NAMED("RefStateManager", "RefState switch detected : "
                        << refStateToString(*previousCmd) << " -> " << refStateToString(*currentCmd) << " | "
                        << refStateToString(*getCurrentRefState()) << " : " << getCurrentStrategyTreeName());
            }
        }


        // Check if the keeper changed
        const roboteam_msgs::RefereeData &refData = Referee::get();
        if (refData.us.goalie != lastKnownKeeper) {
            ROS_WARN_STREAM_NAMED("RefStateManager",
                                  "Keeper ID changed from " << lastKnownKeeper << " to " << refData.us.goalie);
            lastKnownKeeper = refData.us.goalie;
            RobotDealer::claimKeeper(
                    lastKnownKeeper); // This is also done in StrategyNode, but its needs to be done before this Terminate call
            // Node question: Why do we need to call the status here again?
            getCurrentChild()->Terminate(getCurrentChild()->getStatus());
            needToInitialize = true;
        }

        // If the current strategy tree needs to be re-initialized
        if (needToInitialize) {
            needToInitialize = false;

            ROS_DEBUG_STREAM_NAMED("RefStateManager", "Initializing current tree : " << getCurrentStrategyTreeName());
            getCurrentChild()->Initialize(); // getCurrentChild() automatically returns right tree based on previousCmd and currentCmd
        }

        // Run the strategy tree
        bt::Node::Status currentStatus = Status::Invalid;
        try {
            currentStatus = getCurrentChild()->Update();
        } catch (const std::out_of_range &e) {
            ROS_ERROR_STREAM_NAMED("RefStateManager", "Exception caught!");
            ROS_ERROR_STREAM_NAMED("RefStateManager", e.what());
        } catch (const std::exception &e) {
            ROS_ERROR_STREAM_NAMED("RefStateManager", "Exception caught!");
            ROS_ERROR_STREAM_NAMED("RefStateManager", e.what());
        } catch (const std::string &e) {
            ROS_ERROR_STREAM_NAMED("RefStateManager", "Exception caught!");
            ROS_ERROR_STREAM_NAMED("RefStateManager", e.c_str());
        } catch (...) {
            ROS_ERROR_STREAM_NAMED("RefStateManager", "General exception caught!");
        }


        // If the current tree is finished, set needToInitialize and finishedOnce
        if (currentStatus == Status::Failure || currentStatus == Status::Success) {
            ROS_DEBUG_STREAM_NAMED("RefStateManager", "Current tree finished : " << getCurrentStrategyTreeName());
            needToInitialize = true;
            finishedOnce = true;
        }

        return bt::Node::Status::Running;
    }

    bool RefStateManager::hasStartedNewStrategy() const {
        return startedNewStrategy;
    }

    /**
     * Returns the current referee state based on previousCmd, currentCmd, and finishedOnce
     * @returns any of the possible referee states including our custom ones (DO_KICKOFF, DO_PENALTY etc.)
     */
    boost::optional <RefGameState> RefStateManager::getCurrentRefState() const {
        std::string previousCmdName = "none yet";
        std::string currentCmdName = "none yet";

        if (currentCmd) {
            currentCmdName = refStateToString(*currentCmd);
        }

        if (previousCmd) {
            previousCmdName = refStateToString(*previousCmd);
        }

        // If there was a specific switch from one state to another (isTwoState). It is
        if (currentCmd && Referee::isTwoState(previousCmd, *currentCmd)) {
            // If we are still in the TwoState
            if (!finishedOnce) {
                // getFirstState returns the state that the specific switch leads to, e.g. DO_KICKOFF
                if (auto targetStateOpt = Referee::getFirstState(previousCmd, *currentCmd)) {
                    return *targetStateOpt;

                    // If getFirstState returns nothing, there is something wrong..
                } else {
                    ROS_ERROR_STREAM_NAMED("RefStateSwitch", "PreviousCmd and currentCmd are twoState states, "
                            << "but getFirstState returned nothing! Something is wrong in the twoState mapping in Referee.cpp!"
                            << "PreviousCmd : " << previousCmdName.c_str()
                            << "currentCmd : " << currentCmdName.c_str()
                    );

                    return boost::none;
                }

                // If we finished the TwoState, return NORMAL_START as the default state
            } else {
                return RefGameState::NORMAL_START;
            }

            // If there is no current command, return none
        } else if (!currentCmd) {
            return boost::none;
            // Return the current RefState
        } else {
            return *currentCmd;
        }
    }
    std::string RefStateManager::getCurrentStrategyTreeName() const {
        if (auto refStateOpt = getCurrentRefState()) {
            auto const it = StrategyMapper::MAPPING.find(*refStateOpt);
            if (it != StrategyMapper::MAPPING.end()) {
                return *it->second;
            }
        }
        return "Current strategy not found? Check if a RefState was sent and if all trees were set correctly.";
    }

    bt::Node::Ptr RefStateManager::getCurrentChild() {
        std::string previousCmdName = "none yet";
        std::string currentCmdName = "none yet";

        if (currentCmd) {
            currentCmdName = refStateToString(*currentCmd);
        }

        if (previousCmd) {
            previousCmdName = refStateToString(*previousCmd);
        }

        if (auto targetStateOpt = getCurrentRefState()) {
            RefGameState state=*targetStateOpt;
            auto stateIt = refStateStrategies.find(*targetStateOpt);
            if (stateIt != refStateStrategies.end()) {
                return stateIt->second;
            } else {
                ROS_ERROR_NAMED("RefStateManager", "No strategy tree found! Previouscmd: %s, currentCmd: %s",
                                previousCmdName.c_str(),
                                currentCmdName.c_str()
                );

                return nullptr;
            }
        } else {
            return nullptr;
        }
    }

    void RefStateManager::Terminate(Status) {
        ROS_FATAL_NAMED("RefStateManager", "TERMINATING THE REF STATE MANAGER IS NOT SUPPORTED!");
    }

    std::string RefStateManager::node_name() {
        return "RefStateManager";
    }
} // ai
} // rtt
