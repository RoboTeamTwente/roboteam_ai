//
// Created by john on 3/9/20.
//

#include <gtest/gtest.h>
#include <test/helpers/WorldHelper.h>

#include <stp/PlayChecker.hpp>
#include <stp/PlayDecider.hpp>

class trueInvariant : public rtt::ai::stp::invariant::BaseInvariant {
    double metricCheck(rtt::world_new::view::WorldDataView world, const rtt::ai::Field *field) const noexcept override { return 1; }
};

class falseInvariant : public rtt::ai::stp::invariant::BaseInvariant {
    double metricCheck(rtt::world_new::view::WorldDataView world, const rtt::ai::Field *field) const noexcept override { return 0; }
};

class AlwaysValid : public rtt::ai::stp::Play {
   public:
    AlwaysValid() : Play() {
        startPlayInvariants.emplace_back(std::make_unique<trueInvariant>());
    }

    uint8_t score(rtt::world_new::World *world) noexcept override { return 100; }

    rtt::ai::Dealer::FlagMap decideRoleFlags() const noexcept override { return {}; }

    void calculateInfoForRoles() noexcept override {}

    bool shouldRoleSkipEndTactic() override { return false; }

    const char *getName() override { return "Always Valid Play"; }
};

class AlwaysFalse : public rtt::ai::stp::Play {
   public:
    AlwaysFalse() : Play() {
        startPlayInvariants.emplace_back(std::make_unique<falseInvariant>());
    }

    uint8_t score(rtt::world_new::World *world) noexcept override { return 0; }

    rtt::ai::Dealer::FlagMap decideRoleFlags() const noexcept override { return {}; }

    void calculateInfoForRoles() noexcept override {}

    bool shouldRoleSkipEndTactic() override { return false; }

    const char *getName() override { return "Always Invalid Play"; }
};

class AnotherAlwaysTrue : public AlwaysValid {
    using AlwaysValid::AlwaysValid;
};

TEST(PlayCheckerTests, testHighestScore) {
    using namespace rtt::ai::stp;

    PlayChecker checker{};
    std::vector<std::unique_ptr<Play>> plays;
    plays.emplace_back(std::make_unique<AlwaysValid>());
    plays.emplace_back(std::make_unique<AlwaysFalse>());
    plays.emplace_back(std::make_unique<AnotherAlwaysTrue>());

    auto instance = rtt::world_new::World::instance();

    proto::GeometryFieldSize size {};
    size.set_field_length(250);

    auto world_msg = testhelpers::WorldHelper::getWorldMsg(5, 7, true, size);
    rtt::ai::Field field{};

    instance->updateWorld(world_msg);
    instance->updateField(field);

    checker.update(instance);
    checker.setPlays(plays);

    PlayDecider decider{};
    auto play = decider.decideBestPlay(rtt::world_new::World::instance(), checker.getValidPlays());

    ASSERT_TRUE(dynamic_cast<AlwaysValid *>(play));
    ASSERT_EQ(play->score(rtt::world_new::World::instance()), 100);
}
