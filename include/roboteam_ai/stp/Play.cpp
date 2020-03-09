//
// Created by john on 3/9/20.
//

#include "Play.hpp"

namespace rtt::ai::stp {
    Status Play::update(const rtt::ai::stp::TacticInfo &info) noexcept {
        constexpr static size_t ENUM_COUNT = 4;
        std::array<size_t, ENUM_COUNT> count{};
        std::fill(count.begin(), count.end(), 0);
        for (auto& each : roles) {
            auto index = static_cast<size_t>(each->update(info));
            count[index] += 1;
        }

        if (count[static_cast<size_t>(Status::Success)] == ROBOT_COUNT) {
            return Status::Success;
        }

        if (count[static_cast<size_t>(Status::Failure)]) {
            return Status::Failure;
        }

        if (count[static_cast<size_t>(Status::Waiting)]) {
            return Status::Waiting;
        }

        return Status::Running;
    }
}
