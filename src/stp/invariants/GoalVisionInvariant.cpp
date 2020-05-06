//
// Created by timovdk on 4/24/20.
//

#include "stp/invariants/GoalVisionInvariant.h"

namespace rtt::ai::stp::invariant {
GoalVisionInvariant::GoalVisionInvariant() noexcept {
    /**
     * Creates a piecewise linear function that looks as follows:
     *
     * (0,255)  |            XXXX
     *          |         XXXX
     *          |      XXXX
     *          |   XXXX
     *   (0,0)  |XXXX---------
     *              (visibility in %)
     *
     * For 0% visibility return 0, for 100% visibility return 255
     */
    piecewiseLinearFunction = nativeformat::param::createParam(control_constants::FUZZY_FALSE, control_constants::FUZZY_TRUE, control_constants::FUZZY_FALSE, "goalVision");
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_FALSE, 0.0);
    // end_x is the maximum visibility percentage; 100
    piecewiseLinearFunction->linearRampToYAtX(control_constants::FUZZY_TRUE, 100);
    piecewiseLinearFunction->setYAtX(control_constants::FUZZY_TRUE, 100);
}

uint8_t GoalVisionInvariant::metricCheck(world_new::view::WorldDataView world, const world::Field* field) const noexcept {
    auto& us = world.getUs();
    std::vector<double> visibilities{};
    visibilities.reserve(control_constants::MAX_ROBOT_COUNT);

    std::transform(us.begin(), us.end(), std::back_inserter(visibilities), [&](auto& robot) {
        return FieldComputations::getPercentageOfGoalVisibleFromPoint(*field, false, robot.get()->getPos(), world, robot.get()->getId(), true);
    });

    return calculateMetric(*std::min_element(visibilities.begin(), visibilities.end()));
}

uint8_t GoalVisionInvariant::calculateMetric(const double& x) const noexcept { return piecewiseLinearFunction->yForX(x); }
}  // namespace rtt::ai::stp::invariant