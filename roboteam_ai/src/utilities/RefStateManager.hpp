//
// Created by rolf on 23-10-18.
//

#ifndef ROBOTEAM_AI_REFSTATEMANAGER_HPP
#define ROBOTEAM_AI_REFSTATEMANAGER_HPP
#include "../../src/bt/bt.hpp"
#include "Referee.hpp"
#include "World.h"
#include "RobotDealer.h"
#include "boost/optional.hpp"
namespace rtt{
namespace ai{

/**
 * \class RefStateSwitch
 * \brief Top-level node which selects the correct strategy tree to use based on the current ref state.
 */
class RefStateManager : public bt::Composite{
public:
    RefStateManager();

    bt::Node::Status Update() override;

    void Terminate(Status s) override;

    std::string node_name() override;

    bt::Node::Ptr getCurrentChild();
    std::string getCurrentStrategyTreeName() const;
    boost::optional<RefGameState> getCurrentRefState() const;
    bool hasStartedNewStrategy() const;

    void AddStrategy(RefGameState refState, Node::Ptr child);

private:
    boost::optional<RefGameState> previousCmd;
    boost::optional<RefGameState> currentCmd;
    bool finishedOnce;
    bool needToInitialize;
    bool startedNewStrategy;
    unsigned lastKnownBotCount;
    int lastKnownKeeper = 0;

    std::map<RefGameState, Node::Ptr> refStateStrategies;
};
}
}
#endif //ROBOTEAM_AI_REFSTATEMANAGER_HPP
