#include <utility>

#include <utility>

//
// Created by thijs on 2-5-19.
//

#ifndef ROBOTEAM_AI_REPOBUFFER_H
#define ROBOTEAM_AI_REPOBUFFER_H

#include <iostream>
#include <fstream>
#include <string>
#include <gtest/gtest_prod.h>
#include <map>
#include "TreeInterpreter.h"
#include "BTImport.h"

namespace repo {

class Repo {
    public:
        Repo() = default;
        Repo(std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo,
            std::map<std::string, bt::Node::Ptr> tacticsRepo,
            std::map<std::string, bt::BehaviorTree::Ptr> keeperRepo)
            :strategyRepo(std::move(strategyRepo)), tacticsRepo(std::move(tacticsRepo)), keeperRepo(std::move(keeperRepo)) {};

        std::map<std::string, bt::BehaviorTree::Ptr> strategyRepo;
        std::map<std::string, bt::Node::Ptr> tacticsRepo;
        std::map<std::string, bt::BehaviorTree::Ptr> keeperRepo;
};

class RepoBuffer {

    private:
        Repo* repoBuffer;
        unsigned int size;
        int lastIndex;
        int reposFilled;
    public:
        explicit RepoBuffer(unsigned int size = 20) {
            RepoBuffer::size = size;
            repoBuffer = new Repo[size];
            lastIndex = 0;
            reposFilled = 0;
        }

        ~RepoBuffer() {
            delete[] repoBuffer;
        }

        void addNewRepo(const Repo &repo);

        const Repo &getNewRepo();

};

}

#endif //ROBOTEAM_AI_REPOBUFFER_H
