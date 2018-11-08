//
// Created by baris on 05/11/18.
//

#include "DemoTactic.h"
#include "../../utilities/World.h"

namespace bt {

DemoTactic::DemoTactic(std::string name, Blackboard::Ptr blackboard) {

    globalBB = blackboard;
    setName(name);
}

void DemoTactic::setName(std::string newName) {
    name = newName;

}

void DemoTactic::Initialize() {

    while (!claimedRobots) {
        int numberOfRobots = 1;
        std::set<int> ids;
        ids = DemoTactic::askForRandomRobot(numberOfRobots);
        int id = *ids.begin();  // only one robot...
        if (id != -1) {
            claimedRobots = RobotDealer::claimRobotForTactic(id, "testTactic", "testRole");
            robotIDs.insert(id);
        }
    }
}

Node::Status DemoTactic::Update() {
    Node::append_status("[DemoTactic untill success: executing child of type %s]", child->node_name().c_str());
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


std::set<int> DemoTactic::askForRandomRobot(int numberOfRobots) {
    std::set<int> ids;
    for (int i = 0; i < numberOfRobots; i++) {
        ids.insert(RobotDealer::claimRandomRobot());
    }
    if (ids.find(-1) == ids.end()) {
        return ids;
    } else return {-1};
}

} // bt










