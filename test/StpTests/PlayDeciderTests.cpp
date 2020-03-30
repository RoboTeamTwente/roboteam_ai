//
// Created by john on 3/9/20.
//

#include <gtest/gtest.h>

#include <stp/PlayDecider.hpp>
#include <stp/PlayChecker.hpp>

class AlwaysValid : public rtt::ai::stp::Play {
   public:
    AlwaysValid(std::string playName) : Play(playName) {}

    uint8_t score(rtt::world_new::World *world) noexcept override { return 100; }

    rtt::ai::Dealer::FlagMap decideRoleFlags() const noexcept override { return {}; }

    void calculateInfoForRoles() noexcept override {}

    bool isValidPlayToStart(rtt::world_new::World *world) noexcept override { return true; }
    bool shouldRoleSkipEndTactic() override { return false; }
};

class AlwaysFalse : public rtt::ai::stp::Play {
   public:
    AlwaysFalse(std::string playName) : Play(playName) {}
    uint8_t score(rtt::world_new::World *world) noexcept override { return 0; }

    rtt::ai::Dealer::FlagMap decideRoleFlags() const noexcept override { return {}; }

    void calculateInfoForRoles() noexcept override {}

    bool isValidPlayToStart(rtt::world_new::World *world) noexcept override { return false; }
    bool shouldRoleSkipEndTactic() override { return false; }
};

class AnotherAlwaysTrue : public AlwaysValid {
    using AlwaysValid::AlwaysValid;
};

TEST(PlayCheckerTests, testHighestScore) {
    using namespace rtt::ai::stp;

    PlayChecker checker{};
    std::vector<std::unique_ptr<Play>> plays;
    plays.emplace_back(std::make_unique<AlwaysValid>("Always Valid"));
    plays.emplace_back(std::make_unique<AlwaysFalse>("Always False"));
    plays.emplace_back(std::make_unique<AnotherAlwaysTrue>("Also Always Valid"));

    checker.setPlays(plays);

    PlayDecider decider{};
    auto play = decider.decideBestPlay(rtt::world_new::World::instance(), checker.getValidPlays());

    ASSERT_TRUE(dynamic_cast<AlwaysValid *>(play));
    ASSERT_EQ(play->score(rtt::world_new::World::instance()), 100);
}
