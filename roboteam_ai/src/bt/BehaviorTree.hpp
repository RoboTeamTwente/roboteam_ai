#pragma once

#include "Node.hpp"
#include "Blackboard.hpp"

namespace bt {

class BehaviorTree : public Node {
    public:

        using Ptr = std::shared_ptr<BehaviorTree>;

        BehaviorTree();

        BehaviorTree(const Node::Ptr &rootNode);

        BehaviorTree(const Blackboard::Ptr &shared);

        std::string node_name() override;


        ~BehaviorTree();

        Status Update() override;

        void Terminate(Status s) override;

        void SetRoot(const Node::Ptr &node);

        Node::Ptr GetRoot();

        void SetGlobalBlackboard(const Blackboard::Ptr &globalBB);

    private:
        Node::Ptr root = nullptr;
        Blackboard::Ptr globalBB = nullptr;


};

} // bt
