

//
// Created by baris on 04/10/18.
//

#ifndef ROBOTEAM_AI_BTFACTORY_H
#define ROBOTEAM_AI_BTFACTORY_H

#include <roboteam_ai/src/bt/BehaviorTree.hpp>
#include <map>

class BTFactory {
   static std::mutex keeperTreeMutex;

    public:
        static void makeTrees();

        static bt::BehaviorTree::Ptr getTree(std::string treeName);

        static std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;

        static std::map<std::string, bt::Node::Ptr> tacticsRepo;

        static std::map<std::string, bt::BehaviorTree::Ptr> keeperRepo;

        static std::string getCurrentTree();

        static bt::BehaviorTree::Ptr getKeeperTree();

        static void setCurrentTree(const std::string &currentTree);

        static void setKeeperTree(const std::string &keeperTree);

        static void halt();

        static std::string getKeeperTreeName();

    private:
        static std::string currentTree;
        static std::string keeperTree;
};

#endif //ROBOTEAM_AI_BTFACTORY_H

