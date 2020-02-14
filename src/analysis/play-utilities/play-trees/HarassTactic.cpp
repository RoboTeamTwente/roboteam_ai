//
// Created by jessevw on 14.02.20.
//

#include "HarassTactic.h"


bt::Node::Status bt::HarassTactic::update() {
    return Status::Waiting;
}

void bt::HarassTactic::build() {
    auto lbb = std::make_shared<Blackboard>();
    halt = std::make_shared<rtt::ai::Halt>("halt", lbb);
    gtp = std::make_shared<rtt::ai::SkillGoToPos>("gotopos", lbb);
    this->addChild(gtp);
    this->addChild(halt);
}

bt::HarassTactic::HarassTactic() {
    build();
}
