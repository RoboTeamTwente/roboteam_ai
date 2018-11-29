//
// Created by baris on 29-11-18.
//

#ifndef ROBOTEAM_AI_DEFAULTTACTIC_H
#define ROBOTEAM_AI_DEFAULTTACTIC_H

#include "../Tactic.h"
namespace bt {

class DefaultTactic : public Tactic {

        // ------ EDIT ------

        std::vector<std::string> roleNames = {"this", "that", "these", "those"};


        // ------ DO NOT EDIT-----

    public:

        std::string name;

        int robotsNeeded = static_cast<int>(roleNames.size());

        int claimedRobots = 0;

        std::set<int> robotIDs;

        std::map<std::string, robotType> robots;

        DefaultTactic(std::string name, Blackboard::Ptr blackboard, std::map<std::string, robotType> robots);

        void setName(std::string newName);

        void initialize() override;

        Node::Status update() override;

        void terminate(Status s) override;

        std::string node_name() override;

        void claimRobots();

};
}

#endif //ROBOTEAM_AI_DEFAULTTACTIC_H
