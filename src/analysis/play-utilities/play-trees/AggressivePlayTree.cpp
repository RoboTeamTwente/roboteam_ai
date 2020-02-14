//
// Created by jessevw on 13.02.20.
//
#include <include/roboteam_ai/world/World.h>
#include <include/roboteam_ai/utilities/IOManager.h>
#include "AggressivePlayTree.h"
#include "world/Field.h"
namespace bt {
    void AggressivePlayTree::build() {
        this->harassTactic.build();
    }

    AggressivePlayTree::AggressivePlayTree() : BehaviorTree() {

    }

    void AggressivePlayTree::setParameters() {

    }

    void AggressivePlayTree::execute() {
        this->setParameters();
        this->GetRoot()->tick(world, field);
    }

}