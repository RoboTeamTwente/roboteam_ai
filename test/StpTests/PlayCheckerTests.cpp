//
// Created by john on 3/9/20.
//

#include <gtest/gtest.h>

#include <stp/PlayChecker.hpp>

class AlwaysValid : public rtt::ai::stp::Play {
public:
    bool isValidPlay(rtt::world_new::World *world) noexcept override {
        return true;
    }

    uint8_t score(rtt::world_new::World *world) noexcept override {
        return 100;
    }
};

class AlwaysFalse : public rtt::ai::stp::Play {
    bool isValidPlay(rtt::world_new::World *world) noexcept override {
        return false;
    }

    uint8_t score(rtt::world_new::World *world) noexcept override {
        return 0;
    }
};

class AnotherAlwaysTrue : public AlwaysValid{
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