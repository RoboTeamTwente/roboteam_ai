#pragma once

#include "Node.hpp"
#include "Blackboard.hpp"

namespace bt {

    class BehaviorTree : public Node {
    public:
        BehaviorTree();

        BehaviorTree(const Node::Ptr &rootNode);

        BehaviorTree(const Blackboard::Ptr &shared);

        ~BehaviorTree();

        Status Update() override;

        void Terminate(Status s) override;

        void SetRoot(const Node::Ptr &node);

        Node::Ptr GetRoot();

        Blackboard::Ptr GetBlackboard() const;

        Blackboard::Ptr GetSharedBlackboard() const;

        void SetSharedBlackboard(const Blackboard::Ptr &shared);

    private:
        Node::Ptr root = nullptr;
        Blackboard::Ptr blackboard = nullptr;
        Blackboard::Ptr sharedBlackboard = nullptr;

    };

} // bt
