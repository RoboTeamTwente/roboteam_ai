//
// Created by baris on 10-4-19.
//

#ifndef ROBOTEAM_AI_ROLEDIVIDER_H
#define ROBOTEAM_AI_ROLEDIVIDER_H
#include "Node.hpp"
namespace bt {

class RoleDivider : public Node {

    public:
        std::vector<bt::Node::Ptr> children;
        void addChild(bt::Node::Ptr) override;
        std::vector<Node::Ptr> getChildren() override;
        std::string node_name() override;
        std::string name;
        void giveProperty(std::string a, std::string b) override;
        void initialize() override;
        Status update() override;
        void terminate(Status s) override;


};
}
#endif //ROBOTEAM_AI_ROLEDIVIDER_H
