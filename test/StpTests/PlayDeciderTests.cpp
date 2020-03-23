//
// Created by john on 3/9/20.
//

#include <gtest/gtest.h>

#include <stp/PlayChecker.hpp>
#include <include/roboteam_ai/stp/PlayDecider.hpp>

class AlwaysValid : public rtt::ai::stp::Play {
public:
    uint8_t score(rtt::world_new::World *world) noexcept override {
        return 100;
    }

    void assignRoles() noexcept override {
    }

    virtual void calculateInfoForRoles() noexcept {
    }

    bool isValidPlayToStart(rtt::world_new::World *world) noexcept override{return true;}
    bool isValidPlayToKeep(rtt::world_new::World *world) noexcept override{return true;}
    bool shouldRoleSkipEndTactic() override { return false;}
};
class AlwaysFalse : public rtt::ai::stp::Play {

    uint8_t score(rtt::world_new::World *world) noexcept override {
        return 0;
    }

    void assignRoles() noexcept override {
    }

    virtual void calculateInfoForRoles() noexcept {
    }

    bool isValidPlayToStart(rtt::world_new::World *world) noexcept override{return false;}
    bool isValidPlayToKeep(rtt::world_new::World *world) noexcept override{return false;}
    bool shouldRoleSkipEndTactic() override { return false;}
};

class AnotherAlwaysTrue : public AlwaysValid{
    using AlwaysValid::AlwaysValid;
};

TEST(PlayCheckerTests, testHighestScore) {
    using namespace rtt::ai::stp;

    PlayChecker checker{};
    std::vector<std::unique_ptr<Play>> plays;
    plays.emplace_back(std::make_unique<AlwaysValid>());
    plays.emplace_back(std::make_unique<AlwaysFalse>());
    plays.emplace_back(std::make_unique<AnotherAlwaysTrue>());

    checker.setPlays(plays);

    PlayDecider decider{};
    auto play = decider.decideBestPlay(rtt::world_new::World::instance(), checker.getValidPlays());

    ASSERT_TRUE(dynamic_cast<AlwaysValid*>(play));
    ASSERT_EQ(play->score(rtt::world_new::World::instance()), 100);
}
