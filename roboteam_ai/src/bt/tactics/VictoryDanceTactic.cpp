//
// Created by thijs on 15-11-18.
//

#include "VictoryDanceTactic.h"
#include "../../utilities/World.h"
#include <utility>

namespace bt {

VictoryDanceTactic::VictoryDanceTactic(std::string name, bt::Blackboard::Ptr blackboard){
    globalBB = std::move(blackboard);
    setName(std::move(name));
}

void VictoryDanceTactic::setName(std::string newName) {
    name = std::move(newName);
}

void VictoryDanceTactic::Initialize() {

    std::vector<std::string> roleNames = {"victory1"};
    for (auto &roleName : roleNames) {
        while (!claimedRobots) {
            std::set<int> ids;
            ids = RobotDealer::getAvailableRobots();
            if (!ids.empty()) {
                auto id = *ids.begin();  // only one robot..
                std::pair<int, std::string> idName = {id, roleName};
                claimedRobots = RobotDealer::claimRobotForTactic(idName, "victoryDanceTactic");
                robotIDs.insert(id);
            }
        }
        claimedRobots = false;
    }
}

Node::Status VictoryDanceTactic::Update() {
    auto status = child->Tick();

    if (status == Status::Success) {
        return Status::Success;
    }
    else if (status == Status::Invalid) {
        return Status::Failure;
    }
    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}

std::string VictoryDanceTactic::node_name() {
    return "Parallel sequence tactic";
}

} //bt
