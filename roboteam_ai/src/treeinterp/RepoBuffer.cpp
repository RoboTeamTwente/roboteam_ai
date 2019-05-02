//
// Created by thijs on 2-5-19.
//

#include "RepoBuffer.h"
#include "BTFactory.h"

namespace repo {

void RepoBuffer::addNewRepo(const Repo &repo) {
    repoBuffer[lastIndex --] = repo;
    reposFilled ++;
    if (lastIndex < 0) lastIndex = size - 1;
}

const Repo &RepoBuffer::getNewRepo() {
    if (reposFilled <= 0) {
        BTFactory::createNewTrees();

    }
    reposFilled --;
    unsigned int location = lastIndex + 1;
    location %= size;
    return repoBuffer[location];
}

}