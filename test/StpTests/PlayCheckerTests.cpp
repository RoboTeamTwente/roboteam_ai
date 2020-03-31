//
// Created by john on 3/9/20.
//

#include <gtest/gtest.h>

#include <stp/PlayChecker.hpp>

class AlwaysValid : public rtt::ai::stp::Play {
   public:
    AlwaysValid() : Play() {}

    uint8_t score(rtt::world_new::World *world) noexcept override { return 100; }

    rtt::ai::Dealer::FlagMap decideRoleFlags() const noexcept override { return {}; }
    void calculateInfoForRoles() noexcept override {}

    bool shouldRoleSkipEndTactic() override { return false; }
    bool isValidPlayToStart(rtt::world_new::World *world) noexcept override { return true; }

    const char *getName() override { return "Always Valid Play"; }
};

class AlwaysFalse : public rtt::ai::stp::Play {
   public:
    AlwaysFalse() : Play() {}
    uint8_t score(rtt::world_new::World *world) noexcept override { return 0; }

    rtt::ai::Dealer::FlagMap decideRoleFlags() const noexcept override { return {}; }

    void calculateInfoForRoles() noexcept override {}

    bool shouldRoleSkipEndTactic() override { return false; }

    bool isValidPlayToStart(rtt::world_new::World *world) noexcept override { return false; }

    const char *getName() override { return "Always Invalid Play"; }
};

class AnotherAlwaysTrue : public AlwaysValid {
    using AlwaysValid::AlwaysValid;
};

TEST(PlayCheckerTests, testSetPlays) {
    using namespace rtt::ai::stp;

    PlayChecker checker{};
    std::vector<std::unique_ptr<Play>> plays;
    plays.emplace_back(std::make_unique<AlwaysValid>());
    plays.emplace_back(std::make_unique<AlwaysFalse>());
    plays.emplace_back(std::make_unique<AnotherAlwaysTrue>());

    checker.setPlays(plays);
}

TEST(PlayCheckerTests, testValidCount) {
    using namespace rtt::ai::stp;

    PlayChecker checker{};
    std::vector<std::unique_ptr<Play>> plays;
    plays.emplace_back(std::make_unique<AlwaysValid>());
    plays.emplace_back(std::make_unique<AlwaysFalse>());
    plays.emplace_back(std::make_unique<AnotherAlwaysTrue>());

    checker.setPlays(plays);

    ASSERT_EQ(checker.getValidPlays().size(), 2);
}
