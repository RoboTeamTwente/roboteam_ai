//
// Created by rolf on 21/11/18.
//

#include "grsimTestTactic.h"
namespace bt{
grsimTestTactic::grsimTestTactic(std::string name, bt::Blackboard::Ptr blackboard) {
    globalBB = std::move(blackboard);
    setName(std::move(name));
}
void grsimTestTactic::setName(std::string newName) {
    name = std::move(newName);
}
void grsimTestTactic::initialize() {
    std::vector<std::string> roleNames = {"grsim1"};
    while (claimedRobots < roleNames.size()) {
        robotIDs.insert(dealer::claimRobotForTactic(robotType::random, roleNames[claimedRobots], "grsimTestTactic"));
        if (robotIDs.find(-1) == robotIDs.end()) claimedRobots++;
        else robotIDs.erase(-1);
    }
}
Node::Status grsimTestTactic::update(){
    auto status = child->tick();
    if (status == Status::Success) {
        return Status::Success;
    }
    else /* if (status == Status::Failure || status == Status::Running) */ {
        // If the status was anything but success/invalid, keep running
        return Status::Running;
    }
}
std::string grsimTestTactic::node_name() {
        return "grsimTestTactic";
    }
}//bt