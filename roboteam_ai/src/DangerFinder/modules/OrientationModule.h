#pragma once

#include "DangerModule.h"

namespace rtt {
namespace ai {
namespace dangerfinder {

/**
 * \class OrientationModule
 * \brief DangerModule which assigns higher scores to robots which are oriented towards our goal.
 */
class OrientationModule final : public DangerModule {
    private:

    public:
        explicit OrientationModule() = default;

        OrientationModule(double factor, double scalar, double danger);

        PartialResult
        calculate(const roboteam_msgs::WorldRobot &bot, const roboteam_msgs::World &world) override;

    private:
        double factor;
        double scalar;
        double danger;

};

} // dangerfinder
} // ai
} // rtt
